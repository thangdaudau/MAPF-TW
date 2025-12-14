#!/usr/bin/env python3
import yaml
import pygame
import numpy as np
import argparse
import time
import sys
import os
from PIL import Image

Colors = {
    'agent': (255, 165, 0),      # Orange
    'goal': (255, 165, 0, 128),  # Orange with alpha
    'obstacle': (255, 0, 0),      # Red
    'boundary': (255, 0, 0),      # Red
    'background': (255, 255, 255), # White
    'text': (0, 0, 0),            # Black
    'button': (144, 238, 144),    # Light green
    'button_hover': (240, 128, 128), # Light coral
    'button_export': (173, 216, 230), # Light blue
    'button_export_hover': (135, 206, 250), # Light sky blue
    'slider_active': (135, 206, 235), # Sky blue
    'slider_track': (211, 211, 211)   # Light gray
}

class Button:
    def __init__(self, x, y, width, height, text, color, hover_color):
        """x, y, width, height are in normalized coordinates [0, 1]"""
        self.norm_rect = (x, y, width, height)
        self.text = text
        self.color = color
        self.hover_color = hover_color
        self.is_hovered = False
        self.rect = None  # Will be set by update_rect
        
    def update_rect(self, screen_width, screen_height):
        """Convert normalized coordinates to pixel coordinates"""
        x = int(self.norm_rect[0] * screen_width)
        y = int(self.norm_rect[1] * screen_height)
        w = int(self.norm_rect[2] * screen_width)
        h = int(self.norm_rect[3] * screen_height)
        self.rect = pygame.Rect(x, y, w, h)
        
    def draw(self, screen, font):
        color = self.hover_color if self.is_hovered else self.color
        pygame.draw.rect(screen, color, self.rect)
        pygame.draw.rect(screen, (0, 0, 0), self.rect, 2)
        
        text_surface = font.render(self.text, True, (0, 0, 0))
        text_rect = text_surface.get_rect(center=self.rect.center)
        screen.blit(text_surface, text_rect)
        
    def handle_event(self, event):
        if event.type == pygame.MOUSEMOTION:
            self.is_hovered = self.rect.collidepoint(event.pos)
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if self.is_hovered:
                return True
        return False

class Slider:
    def __init__(self, x, y, width, height, min_val, max_val, initial_val):
        """x, y, width, height are in normalized coordinates [0, 1]"""
        self.norm_rect = (x, y, width, height)
        self.min_val = min_val
        self.max_val = max_val
        self.value = initial_val
        self.dragging = False
        self.norm_handle_radius = 0.015  # Normalized radius
        self.rect = None
        self.handle_radius = 0
        
    def update_rect(self, screen_width, screen_height):
        """Convert normalized coordinates to pixel coordinates"""
        x = int(self.norm_rect[0] * screen_width)
        y = int(self.norm_rect[1] * screen_height)
        w = int(self.norm_rect[2] * screen_width)
        h = int(self.norm_rect[3] * screen_height)
        self.rect = pygame.Rect(x, y, w, h)
        self.handle_radius = int(self.norm_handle_radius * screen_height)
        
    def draw(self, screen):
        # Draw track
        pygame.draw.rect(screen, Colors['slider_track'], self.rect)
        pygame.draw.rect(screen, (0, 0, 0), self.rect, 1)
        
        # Draw handle
        handle_x = self.rect.x + (self.value - self.min_val) / (self.max_val - self.min_val) * self.rect.width
        handle_y = self.rect.centery
        pygame.draw.circle(screen, Colors['slider_active'], (int(handle_x), handle_y), self.handle_radius)
        pygame.draw.circle(screen, (0, 0, 0), (int(handle_x), handle_y), self.handle_radius, 2)
        
    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            handle_x = self.rect.x + (self.value - self.min_val) / (self.max_val - self.min_val) * self.rect.width
            handle_y = self.rect.centery
            distance = ((event.pos[0] - handle_x) ** 2 + (event.pos[1] - handle_y) ** 2) ** 0.5
            if distance <= self.handle_radius:
                self.dragging = True
        elif event.type == pygame.MOUSEBUTTONUP:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            rel_x = max(0, min(event.pos[0] - self.rect.x, self.rect.width))
            self.value = self.min_val + (rel_x / self.rect.width) * (self.max_val - self.min_val)
            return True
        return False

class Animation:
    def __init__(self, map_data, schedule, output_name):
        pygame.init()
        
        self.map = map_data
        self.schedule = schedule
        self.combined_schedule = self.schedule["schedule"]
        self.output_name = output_name
        
        # Fixed screen dimensions
        self.screen_width = 900
        self.screen_height = 600
        
        self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
        pygame.display.set_caption("Multi-Agent Pathfinding Visualization")
        
        # Normalized layout constants
        self.CONTROL_PANEL_WIDTH = 0.15  # 15% of screen
        self.MAP_AREA_WIDTH = 1.0 - self.CONTROL_PANEL_WIDTH  # 85% of screen
        
        # Calculate map scaling
        self.setup_map_transform()
        
        # Fonts (responsive sizing)
        font_scale = self.screen_height / 650
        self.font_small = pygame.font.Font(None, int(20 * font_scale))
        self.font_medium = pygame.font.Font(None, int(24 * font_scale))
        self.font_large = pygame.font.Font(None, int(32 * font_scale))
        
        # Animation state
        self.current_speed = 1.0
        self.paused = False
        self.virtual_time = 0.0
        self.last_real_time = None
        self.fps = 60
        self.pause_frames = self.fps * 2
        
        # Export state
        self.is_exporting = False
        self.export_frames = []
        self.export_status = ""
        
        # Path tracking state
        self.show_paths = False
        self.show_goal_lines = False
        self.agent_paths = {}  # Dictionary to store path history for each agent
        self.agent_colors = {}  # Dictionary to store random colors for each agent
        
        # Generate random colors for each agent
        import random
        random.seed(42)  # For consistent colors across runs
        for agent_name in self.combined_schedule.keys():
            self.agent_colors[agent_name] = (
                random.randint(0, 255),
                random.randint(0, 255),
                random.randint(0, 255)
            )
        
        # Calculate max time
        self.T = 0
        for agent_name, agent_schedule in self.combined_schedule.items():
            self.T = max(self.T, agent_schedule[-1]["t"])
        
        # Setup controls
        self.setup_controls()
        
        # Clock
        self.clock = pygame.time.Clock()
        

    def setup_map_transform(self):
        """Setup coordinate transformation for map rendering"""
        # Map dimensions are [height, width] in YAML
        # But we treat them as we need: first is vertical extent, second is horizontal
        self.map_height = self.map["map"]["dimensions"][0]
        self.map_width = self.map["map"]["dimensions"][1]
        
        # Available pixel space for map
        available_pixel_width = int(self.MAP_AREA_WIDTH * self.screen_width * 0.95)
        available_pixel_height = int(self.screen_height * 0.95)
        
        # Calculate cell size in pixels to fit the screen
        cell_size_by_width = available_pixel_width / self.map_width
        cell_size_by_height = available_pixel_height / self.map_height
        self.pixel_cell_size = min(cell_size_by_width, cell_size_by_height)
        
        # Pixel map dimensions
        self.pixel_map_width = self.map_width * self.pixel_cell_size
        self.pixel_map_height = self.map_height * self.pixel_cell_size
        
        # Calculate pixel offsets to center the map in the map area
        map_area_pixel_width = int(self.MAP_AREA_WIDTH * self.screen_width)
        self.pixel_map_offset_x = (map_area_pixel_width - self.pixel_map_width) / 2
        self.pixel_map_offset_y = (self.screen_height - self.pixel_map_height) / 2

        
    def norm_to_pixel(self, norm_x, norm_y):
        """Convert normalized coordinates [0, 1] to pixel coordinates"""
        pixel_x = int(norm_x * self.screen_width)
        pixel_y = int(norm_y * self.screen_height)
        return pixel_x, pixel_y
        

    def world_to_pixel(self, x, y):
        """Convert world coordinates to pixel coordinates
        World: x is horizontal (left to right), y is vertical (top to bottom, 0 at top)
        """
        pixel_x = self.pixel_map_offset_x + (x + 0.5) * self.pixel_cell_size
        # Flip Y: map's Y=0 is at bottom, screen's Y=0 is at top
        pixel_y = self.pixel_map_offset_y + (self.map_height - y - 0.5) * self.pixel_cell_size
        return int(pixel_x), int(pixel_y)
    
    def setup_controls(self):
        """Setup UI controls with normalized coordinates"""
        # Control panel position (right side)
        control_start_x = self.MAP_AREA_WIDTH + 0.02
        control_width = self.CONTROL_PANEL_WIDTH - 0.04
        
        # Pause button (normalized coordinates)
        button_y = 0.08
        button_height = 0.06
        self.pause_button = Button(
            control_start_x, button_y, control_width, button_height,
            "||", Colors['button'], Colors['button_hover']
        )
        
        # Speed slider
        slider_y = button_y + button_height + 0.08
        slider_height = 0.03
        self.speed_slider = Slider(
            control_start_x, slider_y, control_width, slider_height,
            0, 10, 1
        )
        
        # Export GIF button
        export_gif_y = slider_y + 0.15
        self.export_gif_button = Button(
            control_start_x, export_gif_y, control_width, button_height,
            "Export GIF", Colors['button_export'], Colors['button_export_hover']
        )
        
        # Export MP4 button
        export_mp4_y = export_gif_y + button_height + 0.02
        self.export_mp4_button = Button(
            control_start_x, export_mp4_y, control_width, button_height,
            "Export MP4", Colors['button_export'], Colors['button_export_hover']
        )
        
        # Show paths toggle button
        show_paths_y = export_mp4_y + button_height + 0.10
        self.show_paths_button = Button(
            control_start_x, show_paths_y, control_width, button_height,
            "Paths: OFF", Colors['button'], Colors['button_hover']
        )
        
        # Show goal lines toggle button
        show_goal_lines_y = show_paths_y + button_height + 0.02
        self.show_goal_lines_button = Button(
            control_start_x, show_goal_lines_y, control_width, button_height,
            "Goals: OFF", Colors['button'], Colors['button_hover']
        )
        
        # Update pixel coordinates
        self.update_ui_elements()
        
    def update_ui_elements(self):
        """Update UI elements pixel coordinates from normalized coordinates"""
        self.pause_button.update_rect(self.screen_width, self.screen_height)
        self.speed_slider.update_rect(self.screen_width, self.screen_height)
        self.export_gif_button.update_rect(self.screen_width, self.screen_height)
        self.export_mp4_button.update_rect(self.screen_width, self.screen_height)
        self.show_paths_button.update_rect(self.screen_width, self.screen_height)
        self.show_goal_lines_button.update_rect(self.screen_width, self.screen_height)
        
    def draw_grid(self):
        """Draw the grid, obstacles, and goals"""
        # Background
        self.screen.fill(Colors['background'])
        
        # Use stored map dimensions
        map_width = self.map_width
        map_height = self.map_height
        
        # Draw boundary
        boundary_rect = pygame.Rect(
            int(self.pixel_map_offset_x), 
            int(self.pixel_map_offset_y), 
            int(self.pixel_map_width), 
            int(self.pixel_map_height)
        )
        pygame.draw.rect(self.screen, Colors['boundary'], boundary_rect, 3)
        
        # Draw vertical grid lines (x direction)
        for x in range(map_width + 1):
            line_x = int(self.pixel_map_offset_x + x * self.pixel_cell_size)
            start_y = int(self.pixel_map_offset_y)
            end_y = int(self.pixel_map_offset_y + self.pixel_map_height)
            pygame.draw.line(self.screen, (200, 200, 200), (line_x, start_y), (line_x, end_y), 1)
        
        # Draw horizontal grid lines (y direction)
        for y in range(map_height + 1):
            line_y = int(self.pixel_map_offset_y + y * self.pixel_cell_size)
            start_x = int(self.pixel_map_offset_x)
            end_x = int(self.pixel_map_offset_x + self.pixel_map_width)
            pygame.draw.line(self.screen, (200, 200, 200), (start_x, line_y), (end_x, line_y), 1)
        
        # Draw obstacles
        for obstacle in self.map["map"]["obstacles"]:
            x, y = obstacle[0], obstacle[1]
            # Obstacle at cell (x, y), need to flip Y for screen coordinates
            pixel_x = int(self.pixel_map_offset_x + x * self.pixel_cell_size)
            pixel_y = int(self.pixel_map_offset_y + (map_height - y - 1) * self.pixel_cell_size)
            rect = pygame.Rect(pixel_x, pixel_y, 
                            int(self.pixel_cell_size), int(self.pixel_cell_size))
            pygame.draw.rect(self.screen, Colors['obstacle'], rect)
            
        # Draw goals
        for i, agent in enumerate(self.map["agents"]):
            gx, gy = agent["goal"]
            # Goal at center of cell (gx, gy)
            center_x, center_y = self.world_to_pixel(gx, gy)
            goal_size = int(self.pixel_cell_size * 0.5)
            screen_x = center_x - goal_size // 2
            screen_y = center_y - goal_size // 2
            rect = pygame.Rect(screen_x, screen_y, goal_size, goal_size)
            
            # Get agent's color (use agent name format)
            agent_name = f"agent{i}"
            goal_color = self.agent_colors.get(agent_name, Colors['agent'])
            
            # Draw semi-transparent goal with agent's color
            s = pygame.Surface((goal_size, goal_size))
            s.set_alpha(128)
            s.fill(goal_color)
            self.screen.blit(s, (screen_x, screen_y))
            pygame.draw.rect(self.screen, Colors['text'], rect, 2)
            
            # Draw goal number in dark color
            text = self.font_small.render(str(i), True, (0, 0, 0))
            text_rect = text.get_rect(center=(center_x, center_y))
            self.screen.blit(text, text_rect)

    def draw_agents(self, t):
        """Draw agents at time t"""
        agent_radius = int(self.pixel_cell_size * 0.3)
        
        for agent_idx, (agent_name, agent_schedule) in enumerate(self.combined_schedule.items()):
            pos = self.get_state(t, agent_schedule)
            screen_x, screen_y = self.world_to_pixel(pos[0], pos[1])
            
            # Get agent's color
            agent_color = self.agent_colors[agent_name]
            
            # Draw line to goal if enabled
            if self.show_goal_lines:
                # Get goal position for this agent
                goal_pos = self.map["agents"][agent_idx]["goal"]
                goal_x, goal_y = self.world_to_pixel(goal_pos[0], goal_pos[1])
                
                # Draw dashed line to goal
                self.draw_dashed_line(self.screen, agent_color, 
                                     (int(screen_x), int(screen_y)), 
                                     (int(goal_x), int(goal_y)), 
                                     width=2, dash_length=10)
            
            # Update path history if tracking is enabled
            if self.show_paths:
                if agent_name not in self.agent_paths:
                    self.agent_paths[agent_name] = []
                self.agent_paths[agent_name].append((screen_x, screen_y))
                
                # Draw path with gradient (lighter at start, darker at end)
                if len(self.agent_paths[agent_name]) > 1:
                    path_points = self.agent_paths[agent_name]
                    total_points = len(path_points)
                    
                    # Draw segments with increasing opacity
                    for i in range(len(path_points) - 1):
                        # Calculate opacity: start at 30, end at 255
                        progress = (i + 1) / total_points
                        opacity = int(30 + (255 - 30) * progress)
                        
                        # Create color with calculated opacity
                        segment_color = (
                            int(agent_color[0] * opacity / 255),
                            int(agent_color[1] * opacity / 255),
                            int(agent_color[2] * opacity / 255)
                        )
                        
                        # Draw line segment
                        pygame.draw.line(self.screen, segment_color, 
                                       path_points[i], path_points[i + 1], 3)
            
            # Draw agent circle with its unique color
            pygame.draw.circle(self.screen, agent_color, 
                             (int(screen_x), int(screen_y)), agent_radius)
            pygame.draw.circle(self.screen, Colors['text'], 
                             (int(screen_x), int(screen_y)), agent_radius, 2)
            
            # Draw agent name
            agent_num = agent_name.replace('agent', '')
            text = self.font_small.render(agent_num, True, Colors['text'])
            text_rect = text.get_rect(center=(screen_x, screen_y))
            self.screen.blit(text, text_rect)
    
    def draw_dashed_line(self, surface, color, start_pos, end_pos, width=1, dash_length=10):
        """Draw a dashed line between two points"""
        x1, y1 = start_pos
        x2, y2 = end_pos
        
        # Calculate distance and direction
        dx = x2 - x1
        dy = y2 - y1
        distance = (dx**2 + dy**2)**0.5
        
        if distance == 0:
            return
        
        # Normalize direction
        dx /= distance
        dy /= distance
        
        # Draw dashes
        current_distance = 0
        draw_dash = True
        
        while current_distance < distance:
            next_distance = min(current_distance + dash_length, distance)
            
            if draw_dash:
                start_x = int(x1 + dx * current_distance)
                start_y = int(y1 + dy * current_distance)
                end_x = int(x1 + dx * next_distance)
                end_y = int(y1 + dy * next_distance)
                pygame.draw.line(surface, color, (start_x, start_y), (end_x, end_y), width)
            
            current_distance = next_distance
            draw_dash = not draw_dash
    
    def draw_controls(self):
        """Draw control panel"""
        # Control panel background (normalized coordinates)
        control_start_norm_x = self.MAP_AREA_WIDTH
        control_x, _ = self.norm_to_pixel(control_start_norm_x, 0)
        control_width = self.screen_width - control_x
        
        control_rect = pygame.Rect(control_x, 0, control_width, self.screen_height)
        pygame.draw.rect(self.screen, (240, 240, 240), control_rect)
        pygame.draw.line(self.screen, (0, 0, 0), (control_x, 0), (control_x, self.screen_height), 2)
        
        # Draw pause button
        self.pause_button.draw(self.screen, self.font_large)
        
        # Speed label (normalized position)
        label_norm_y = self.pause_button.norm_rect[1] + self.pause_button.norm_rect[3] + 0.03
        label_x, label_y = self.norm_to_pixel(control_start_norm_x + self.CONTROL_PANEL_WIDTH / 2, 
                                               label_norm_y)
        speed_label = self.font_small.render("Speed", True, Colors['text'])
        speed_label_rect = speed_label.get_rect(center=(label_x, label_y))
        self.screen.blit(speed_label, speed_label_rect)
        
        # Draw speed slider
        self.speed_slider.draw(self.screen)
        
        # Speed value (normalized position)
        value_norm_y = self.speed_slider.norm_rect[1] + self.speed_slider.norm_rect[3] + 0.03
        value_x, value_y = self.norm_to_pixel(control_start_norm_x + self.CONTROL_PANEL_WIDTH / 2,
                                               value_norm_y)
        speed_text = self.font_medium.render(f"{self.current_speed:.1f}x", True, Colors['text'])
        speed_text_rect = speed_text.get_rect(center=(value_x, value_y))
        self.screen.blit(speed_text, speed_text_rect)
        
        # Draw export buttons
        self.export_gif_button.draw(self.screen, self.font_medium)
        self.export_mp4_button.draw(self.screen, self.font_medium)
        
        # Draw show paths button
        self.show_paths_button.draw(self.screen, self.font_medium)
        
        # Draw show goal lines button
        self.show_goal_lines_button.draw(self.screen, self.font_medium)
        
        # Export status
        if self.export_status:
            status_norm_y = self.export_mp4_button.norm_rect[1] + self.export_mp4_button.norm_rect[3] + 0.05
            status_x, status_y = self.norm_to_pixel(control_start_norm_x + self.CONTROL_PANEL_WIDTH / 2,
                                                     status_norm_y)
            
            # Wrap text if too long
            lines = []
            words = self.export_status.split()
            current_line = ""
            for word in words:
                test_line = current_line + " " + word if current_line else word
                if self.font_small.size(test_line)[0] < control_width - 10:
                    current_line = test_line
                else:
                    if current_line:
                        lines.append(current_line)
                    current_line = word
            if current_line:
                lines.append(current_line)
            
            for i, line in enumerate(lines):
                status_text = self.font_small.render(line, True, (0, 100, 0))
                status_rect = status_text.get_rect(center=(status_x, status_y + i * 20))
                self.screen.blit(status_text, status_rect)
        
        # Time info (normalized position)
        time_x, time_y = self.norm_to_pixel(control_start_norm_x + self.CONTROL_PANEL_WIDTH / 2, 0.94)
        time_text = self.font_small.render(f"Time: {self.virtual_time:.1f}s", True, Colors['text'])
        time_text_rect = time_text.get_rect(center=(time_x, time_y))
        self.screen.blit(time_text, time_text_rect)
    
    def get_state(self, t, schedule):
        """Get agent position at time t"""
        idx = 0
        while idx < len(schedule) and schedule[idx]["t"] < t:
            idx += 1
        
        if idx == 0:
            return np.array([float(schedule[0]["x"]), float(schedule[0]["y"])])
        elif idx < len(schedule):
            pos_last = np.array([float(schedule[idx-1]["x"]), float(schedule[idx-1]["y"])])
            pos_next = np.array([float(schedule[idx]["x"]), float(schedule[idx]["y"])])
            dt = schedule[idx]["t"] - schedule[idx-1]["t"]
            t_interp = (t - schedule[idx-1]["t"]) / dt
            pos = (pos_next - pos_last) * t_interp + pos_last
            return pos
        else:
            return np.array([float(schedule[-1]["x"]), float(schedule[-1]["y"])])
    
    def export_animation(self, format_type='gif'):
        """Export animation to GIF or MP4"""
        self.is_exporting = True
        self.export_frames = []
        self.export_status = f"Exporting {format_type.upper()}..."
        
        # Save original state
        original_paused = self.paused
        original_time = self.virtual_time
        
        # Pause animation during export
        self.paused = True
        
        # Calculate frames - use current speed for export
        export_fps = 30
        # Adjust total time based on current speed
        total_export_time = (self.T + (self.pause_frames / self.fps) * 2) / self.current_speed
        total_frames = int(total_export_time * export_fps)
        
        for frame_idx in range(total_frames):
            t = (frame_idx / export_fps) * self.current_speed
            
            # Calculate simulation time
            if t < self.pause_frames / self.fps:
                sim_t = 0
            else:
                sim_t = t - (self.pause_frames / self.fps)
            
            # Draw frame
            self.draw_grid()
            self.draw_agents(sim_t)
            self.draw_controls()
            
            # Update status
            progress = int((frame_idx + 1) / total_frames * 100)
            self.export_status = f"Rendering: {progress}%"
            self.draw_controls()
            
            pygame.display.flip()
            
            # Capture frame
            frame_data = pygame.surfarray.array3d(self.screen)
            frame_data = np.transpose(frame_data, (1, 0, 2))
            self.export_frames.append(Image.fromarray(frame_data))
            
            # Handle events to keep window responsive
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.is_exporting = False
                    return
        
        # Save to file
        try:
            output_dir = 'exports'
            os.makedirs(output_dir, exist_ok=True)
            
            if format_type == 'gif':
                output_path = os.path.join(output_dir, f'{self.output_name}.gif')
                self.export_status = "Saving GIF..."
                self.draw_controls()
                pygame.display.flip()
                
                self.export_frames[0].save(
                    output_path,
                    save_all=True,
                    append_images=self.export_frames[1:],
                    duration=int(1000/export_fps),
                    loop=0
                )
                self.export_status = f"Saved: {output_path}"
                
            elif format_type == 'mp4':
                try:
                    import cv2
                    output_path = os.path.join(output_dir, f'{self.output_name}.mp4')
                    self.export_status = "Saving MP4..."
                    self.draw_controls()
                    pygame.display.flip()
                    
                    height, width = self.export_frames[0].size[1], self.export_frames[0].size[0]
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    video = cv2.VideoWriter(output_path, fourcc, export_fps, (width, height))
                    
                    for frame in self.export_frames:
                        frame_cv = cv2.cvtColor(np.array(frame), cv2.COLOR_RGB2BGR)
                        video.write(frame_cv)
                    
                    video.release()
                    self.export_status = f"Saved: {output_path}"
                    
                except ImportError:
                    self.export_status = "Error: opencv-python required for MP4"
                    print("Please install: pip install opencv-python")
        
        except Exception as e:
            self.export_status = f"Error: {str(e)}"
            print(f"Export error: {e}")
        
        # Restore original state
        self.paused = original_paused
        self.virtual_time = original_time
        self.is_exporting = False
        self.export_frames = []
    
    def handle_events(self):
        """Handle pygame events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            # Disable controls during export
            if self.is_exporting:
                continue
            
            # Handle pause button
            if self.pause_button.handle_event(event):
                self.paused = not self.paused
                self.pause_button.text = ">" if self.paused else "||"
                if not self.paused:
                    self.last_real_time = None
            
            # Handle speed slider
            if self.speed_slider.handle_event(event):
                self.current_speed = self.speed_slider.value
            
            # Handle export buttons
            if self.export_gif_button.handle_event(event):
                self.export_animation('gif')
            
            if self.export_mp4_button.handle_event(event):
                self.export_animation('mp4')
            
            # Handle show paths button
            if self.show_paths_button.handle_event(event):
                self.show_paths = not self.show_paths
                self.show_paths_button.text = f"Paths: {'ON' if self.show_paths else 'OFF'}"
                if not self.show_paths:
                    # Clear path history when turning off
                    self.agent_paths = {}
            
            # Handle show goal lines button
            if self.show_goal_lines_button.handle_event(event):
                self.show_goal_lines = not self.show_goal_lines
                self.show_goal_lines_button.text = f"Goals: {'ON' if self.show_goal_lines else 'OFF'}"
        
        return True
    
    def update(self):
        """Update animation state"""
        if not self.paused and not self.is_exporting:
            current_real_time = time.time()
            if self.last_real_time is not None:
                delta_real_time = current_real_time - self.last_real_time
                self.virtual_time += delta_real_time * self.current_speed
            self.last_real_time = current_real_time
        
        # Calculate simulation time
        if self.virtual_time < self.pause_frames / self.fps:
            t = 0
        else:
            t = self.virtual_time - (self.pause_frames / self.fps)
        
        # Loop animation
        if t > self.T:
            self.virtual_time = 0
            self.last_real_time = time.time()
            t = 0
            # Clear paths when animation loops
            if self.show_paths:
                self.agent_paths = {}
        
        return t
    
    def run(self):
        """Main animation loop"""
        self.last_real_time = time.time()
        running = True
        
        while running:
            running = self.handle_events()
            
            if not self.is_exporting:
                t = self.update()
                
                # Draw everything
                self.draw_grid()
                self.draw_agents(t)
                self.draw_controls()
                
                pygame.display.flip()
                self.clock.tick(self.fps)
        
        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("name", help="Name of the input/output files (without extension)"
                                     "Example: Enter '6x6x6' to load 'res/6x6x6.input.yaml' and 'res/6x6x6.output.yaml'.")
    args = parser.parse_args()
    
    try:
        with open('res/' + args.name + '.input.yaml') as map_file:
            map_data = yaml.load(map_file, Loader=yaml.FullLoader)
        
        with open('res/' + args.name + '.output.yaml') as states_file:
            schedule = yaml.load(states_file, Loader=yaml.FullLoader)
        
        animation = Animation(map_data, schedule, args.name)
        animation.run()
    except FileNotFoundError as e:
        print(f"Error: Could not find required files in 'res/' directory")
        print(f"Looking for: res/{args.name}.input.yaml and res/{args.name}.output.yaml")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)