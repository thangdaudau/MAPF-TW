#!/usr/bin/env python3
import yaml
import matplotlib
from matplotlib.patches import Circle, Rectangle, Arrow
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import matplotlib.animation as manimation
from matplotlib.widgets import Button, Slider
import argparse
import math
import time

Colors = ['orange', 'blue', 'green']


class BaseAnimation:
    """Base class chứa logic chung cho animation"""
    
    def __init__(self, map_data, schedule):
        self.map = map_data
        self.schedule = schedule
        self.combined_schedule = {}
        self.combined_schedule.update(self.schedule["schedule"])

        # Map dimensions: [height, width] in YAML
        self.map_height = map_data["map"]["dimensions"][0]
        self.map_width = map_data["map"]["dimensions"][1]
        
        aspect = self.map_width / self.map_height

        self.fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
        self.ax = self.fig.add_subplot(111, aspect='equal')
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)
        
        # Tắt các trục tọa độ
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.spines['top'].set_visible(False)
        self.ax.spines['right'].set_visible(False)
        self.ax.spines['bottom'].set_visible(False)
        self.ax.spines['left'].set_visible(False)

        self.patches = []
        self.artists = []
        self.agents = dict()
        self.agent_names = dict()
        
        # Create boundary patch
        # X: 0 to map_width, Y: 0 to map_height (flipped for display)
        xmin = -0.5
        ymin = -0.5
        xmax = self.map_width - 0.5
        ymax = self.map_height - 0.5

        plt.xlim(xmin, xmax)
        plt.ylim(ymin, ymax)

        self.patches.append(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, 
                                     facecolor='none', edgecolor='red', linewidth=2))
        
        # Draw obstacles
        for o in map_data["map"]["obstacles"]:
            x, y = o[0], o[1]
            # Y-flip: obstacle at (x, y) in world -> (x, map_height - y - 1) in display
            display_y = y
            self.patches.append(Rectangle((x - 0.5, display_y - 0.5), 1, 1, 
                                         facecolor='red', edgecolor='red'))

        # Calculate max time
        self.T = 0
        for agent_name, agent_schedule in self.combined_schedule.items():
            self.T = max(self.T, agent_schedule[-1]["t"])
            
        # Draw goals first (semi-transparent)
        for d, i in zip(map_data["agents"], range(0, len(map_data["agents"]))):
            gx, gy = d["goal"]
            # Y-flip for display
            display_y = gy
            self.patches.append(Rectangle((gx - 0.25, display_y - 0.25), 0.5, 0.5,
                                         facecolor=Colors[0], edgecolor='black', alpha=0.5))
            goal_text = self.ax.text(gx, display_y, str(i), ha='center', va='center', 
                                    fontsize=8, color='darkgreen', weight='bold')
            self.artists.append(goal_text)
            
        # Create agents
        for d, i in zip(map_data["agents"], range(0, len(map_data["agents"]))):
            name = d["name"]
            sx, sy = d["start"]
            # Y-flip for display
            display_y = sy
            self.agents[name] = Circle((sx, display_y), 0.3, 
                                      facecolor=Colors[0], edgecolor='black', linewidth=2)
            self.agents[name].original_face_color = Colors[0]
            self.patches.append(self.agents[name])
            
            agent_num = name.replace('agent', '')
            self.agent_names[name] = self.ax.text(sx, display_y, agent_num,
                                                  ha='center', va='center',
                                                  fontsize=8, weight='bold')
            self.artists.append(self.agent_names[name])

        self.fps = 60
        self.pause_frames = self.fps * 2

    def init_func(self):
        """Initialize animation patches and artists"""
        for p in self.patches:
            self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)
        return self.patches + self.artists

    def update_agents(self, t):
        """Update agent positions at time t"""
        for agent_name, agent_schedule in self.combined_schedule.items():
            pos = self.get_state(t, agent_schedule)
            # Y-flip for display
            display_y = pos[1]
            p = (pos[0], display_y)
            self.agents[agent_name].center = p
            self.agent_names[agent_name].set_position(p)

        # Reset all colors
        for _, agent in self.agents.items():
            agent.set_facecolor(agent.original_face_color)

        # Check agent-agent collisions
        agents_array = [agent for _, agent in self.agents.items()]
        for i in range(0, len(agents_array)):
            for j in range(i+1, len(agents_array)):
                d1 = agents_array[i]
                d2 = agents_array[j]
                pos1 = np.array(d1.center)
                pos2 = np.array(d2.center)
                if np.linalg.norm(pos1 - pos2) < 0.7:
                    d1.set_facecolor('red')
                    d2.set_facecolor('red')

        return self.patches + self.artists

    def get_state(self, t, schedule):
        """Get interpolated state at time t (returns world coordinates)"""
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


class AnimationPlayer(BaseAnimation):
    """Class cho việc hiển thị animation tương tác với điều khiển tốc độ"""
    
    def __init__(self, map_data, schedule):
        super().__init__(map_data, schedule)
        
        # Speed control variables
        self.current_speed = 1.0
        self.paused = False
        self.virtual_time = 0.0
        self.last_real_time = None
        
        # Adjust figure layout to make room for controls
        self.fig.subplots_adjust(left=0.15, right=1, bottom=0, top=1)
        
        # Add speed control buttons
        self.setup_speed_controls()
        
        # Create animation
        self.anim = animation.FuncAnimation(
            self.fig, 
            self.animate_func,
            init_func=self.init_func_with_timer,
            frames=10000,  # Large number for continuous animation
            interval=1000.0 / self.fps,
            blit=False,
            repeat=True
        )

    def setup_speed_controls(self):
        """Setup speed control slider and pause button"""
        start_x = 0.02
        start_y = 0.7
        control_width = 0.08
        button_height = 0.06
        slider_height = 0.03
        button_spacing = 0.02
        
        # Pause/Play button
        ax_pause = plt.axes([start_x, start_y, control_width, button_height])
        self.btn_pause = Button(ax_pause, '||', color='lightgreen', hovercolor='lightcoral')
        self.btn_pause.on_clicked(self.toggle_pause)
        
        # Speed label
        self.speed_label = self.fig.text(
            start_x + control_width * 0.5, 
            start_y - button_spacing,
            'Speed', 
            ha='center', va='top', fontsize=7, fontweight='bold'
        )
        
        # Speed slider
        ax_slider = plt.axes([start_x, start_y - button_spacing - 0.05, 
                             control_width, slider_height])
        self.speed_slider = Slider(
            ax=ax_slider,
            label='',
            valmin=0,
            valmax=3,
            valinit=1,
            orientation='horizontal',
            color='skyblue',
            track_color='lightgray'
        )
        self.speed_slider.on_changed(self.update_speed)
        
        # Speed value display
        self.speed_value_label = self.fig.text(
            start_x + control_width * 0.5, 
            start_y - button_spacing - 0.08,
            f'{self.current_speed:.1f}x', 
            ha='center', va='top', fontsize=12, fontweight='bold'
        )

    def toggle_pause(self, event):
        """Toggle pause/play"""
        self.paused = not self.paused
        if self.paused:
            self.btn_pause.label.set_text('>')
        else:
            self.btn_pause.label.set_text('||')
            self.last_real_time = None  # Reset timer when resuming
        self.fig.canvas.draw_idle()

    def update_speed(self, val):
        """Update speed from slider"""
        self.current_speed = val
        self.speed_value_label.set_text(f'{self.current_speed:.1f}x')
        self.fig.canvas.draw_idle()

    def init_func_with_timer(self):
        """Initialize animation with timer reset"""
        self.last_real_time = time.time()
        return self.init_func()

    def animate_func(self, frame):
        """Animation function with speed control"""
        # Update virtual time based on real time and speed
        if not self.paused:
            current_real_time = time.time()
            if self.last_real_time is not None:
                delta_real_time = current_real_time - self.last_real_time
                self.virtual_time += delta_real_time * self.current_speed
            self.last_real_time = current_real_time
        
        # Convert virtual time to simulation time
        if self.virtual_time < self.pause_frames / self.fps:
            t = 0
        else:
            t = self.virtual_time - (self.pause_frames / self.fps)
        
        # Loop animation if it reaches the end
        if t > self.T:
            self.virtual_time = 0
            self.last_real_time = time.time()
            t = 0
        
        return self.update_agents(t)

    def show(self):
        """Display the interactive animation"""
        plt.show()


class VideoRenderer(BaseAnimation):
    """Class cho việc render animation ra video file"""
    
    def __init__(self, map_data, schedule, speed=1):
        super().__init__(map_data, schedule)
        self.speed = speed

    def animate_func(self, frame):
        """Animation function for video rendering"""
        if frame < self.pause_frames:
            t = 0
        else:
            t = (frame - self.pause_frames) / self.fps
        
        return self.update_agents(t)

    def render(self, output_file):
        """Render animation to video file"""
        # Calculate total frames
        total_frames = int(self.T * self.fps) + self.pause_frames
        
        print(f"Rendering video to {output_file}...")
        print(f"Total frames: {total_frames}")
        print(f"Duration: {self.T:.2f}s")
        print(f"Speed multiplier: {self.speed}x")
        
        # Create animation
        anim = animation.FuncAnimation(
            self.fig, 
            self.animate_func,
            init_func=self.init_func,
            frames=total_frames,
            interval=1000.0 / self.fps,
            blit=False,
            repeat=False
        )
        
        # Save to file
        anim.save(
            output_file,
            writer="ffmpeg",
            fps=15 * self.speed,
            dpi=400
        )
        
        print(f"Video saved successfully!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("name", help="Name of the input/output files (without extension)"
                                     "Example: Enter '6x6x6' to load 'res/6x6x6.input.yaml' and 'res/6x6x6.output.yaml'.")
    parser.add_argument('--video', dest='video', default=None, 
                       help="Output video file (or leave empty to show on screen)")
    parser.add_argument("--speed", type=int, default=1, 
                       help="Speedup factor for video rendering")
    args = parser.parse_args()

    try:
        # Load map and schedule
        with open('res/' + args.name + '.input.yaml') as map_file:
            map_data = yaml.load(map_file, Loader=yaml.FullLoader)

        with open('res/' + args.name + '.output.yaml') as states_file:
            schedule = yaml.load(states_file, Loader=yaml.FullLoader)

        if args.video:
            # Render to video
            renderer = VideoRenderer(map_data, schedule, args.speed)
            renderer.render(args.video)
        else:
            # Show interactive player
            player = AnimationPlayer(map_data, schedule)
            player.show()
            
    except FileNotFoundError as e:
        print(f"Error: Could not find required files in 'res/' directory")
        print(f"Looking for: res/{args.name}.input.yaml and res/{args.name}.output.yaml")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()