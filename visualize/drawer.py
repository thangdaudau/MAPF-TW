#!/usr/bin/env python3
import pygame
import yaml
import sys
import random
import os
from tkinter import Tk, filedialog

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
AGENT_COLOR = (255, 165, 0)
GOAL_COLOR = (255, 220, 100)
OBSTACLE_COLOR = (255, 0, 0)
GRAY = (200, 200, 200)
GREEN = (100, 200, 100)
BLUE = (100, 150, 255)

# Ẩn cửa sổ Tkinter chính
root = Tk()
root.withdraw()
root.attributes('-topmost', True)

# --- Khởi tạo ---
pygame.init()
SCREEN_W, SCREEN_H = 1600, 900
screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
pygame.display.set_caption("Map Generator")
font = pygame.font.SysFont(None, 24)
small_font = pygame.font.SysFont(None, 20)

# Hàm chuyển đổi tọa độ chuẩn hóa [0,1] sang pixel
def norm_to_px(x, y, w, h):
    """Chuyển tọa độ chuẩn hóa (0-1) sang pixel"""
    return int(x * SCREEN_W), int(y * SCREEN_H), int(w * SCREEN_W), int(h * SCREEN_H)

def norm_rect(x, y, w, h):
    """Tạo pygame.Rect từ tọa độ chuẩn hóa"""
    px_x, px_y, px_w, px_h = norm_to_px(x, y, w, h)
    return pygame.Rect(px_x, px_y, px_w, px_h)

# --- Màn hình 1: Chọn map ---
class StartScreen:
    def __init__(self):
        self.input_rows = "10"
        self.input_cols = "15"
        self.active_input = None
        
        # UI elements - sử dụng tọa độ chuẩn hóa
        self.input_rows_rect = norm_rect(0.33, 0.25, 0.11, 0.067)
        self.input_cols_rect = norm_rect(0.33, 0.35, 0.11, 0.067)
        self.btn_create_new = norm_rect(0.28, 0.467, 0.22, 0.083)
        self.btn_load_file = norm_rect(0.28, 0.6, 0.22, 0.083)
    
    def draw(self):
        screen.fill(WHITE)
        
        # Title
        title = font.render("MAP GENERATOR", True, BLACK)
        screen.blit(title, (SCREEN_W // 2 - title.get_width() // 2, int(0.083 * SCREEN_H)))
        
        # Create new map section
        subtitle1 = font.render("Create New Map:", True, BLACK)
        screen.blit(subtitle1, (int(0.33 * SCREEN_W), int(0.167 * SCREEN_H)))
        
        # Input rows
        label_rows = font.render("Rows:", True, BLACK)
        screen.blit(label_rows, (int(0.255 * SCREEN_W), int(0.267 * SCREEN_H)))
        color_rows = BLUE if self.active_input == "rows" else BLACK
        pygame.draw.rect(screen, WHITE, self.input_rows_rect)
        pygame.draw.rect(screen, color_rows, self.input_rows_rect, 2)
        text_rows = font.render(self.input_rows, True, BLACK)
        screen.blit(text_rows, (self.input_rows_rect.x + 10, self.input_rows_rect.y + 10))
        
        # Input cols
        label_cols = font.render("Cols:", True, BLACK)
        screen.blit(label_cols, (int(0.255 * SCREEN_W), int(0.367 * SCREEN_H)))
        color_cols = BLUE if self.active_input == "cols" else BLACK
        pygame.draw.rect(screen, WHITE, self.input_cols_rect)
        pygame.draw.rect(screen, color_cols, self.input_cols_rect, 2)
        text_cols = font.render(self.input_cols, True, BLACK)
        screen.blit(text_cols, (self.input_cols_rect.x + 10, self.input_cols_rect.y + 10))
        
        # Buttons
        self.draw_button(self.btn_create_new, "CREATE NEW", GREEN)
        self.draw_button(self.btn_load_file, "LOAD EXISTING MAP", GREEN)
        
        # Instructions
        inst1 = small_font.render("Create a new map with custom dimensions", True, GRAY)
        inst2 = small_font.render("or load an existing .yaml map file", True, GRAY)
        screen.blit(inst1, (SCREEN_W // 2 - inst1.get_width() // 2, int(0.75 * SCREEN_H)))
        screen.blit(inst2, (SCREEN_W // 2 - inst2.get_width() // 2, int(0.792 * SCREEN_H)))
    
    def draw_button(self, rect, text, color):
        pygame.draw.rect(screen, color, rect)
        pygame.draw.rect(screen, BLACK, rect, 2)
        label = font.render(text, True, BLACK)
        screen.blit(label, (rect.centerx - label.get_width() // 2, 
                           rect.centery - label.get_height() // 2))
    
    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            pos = event.pos
            
            # Check input fields
            if self.input_rows_rect.collidepoint(pos):
                self.active_input = "rows"
            elif self.input_cols_rect.collidepoint(pos):
                self.active_input = "cols"
            else:
                self.active_input = None
            
            # Check buttons
            if self.btn_create_new.collidepoint(pos):
                try:
                    rows = int(self.input_rows)
                    cols = int(self.input_cols)
                    if rows > 0 and cols > 0:
                        return ("new", rows, cols)
                except:
                    pass
            
            if self.btn_load_file.collidepoint(pos):
                # Open file dialog
                filepath = filedialog.askopenfilename(
                    title="Select a map file",
                    filetypes=[
                        ("YAML files", "*.yaml"),
                        ("All files", "*.*")
                    ],
                    initialdir=os.getcwd()
                )
                if filepath:
                    return ("load", filepath)
        
        elif event.type == pygame.KEYDOWN and self.active_input:
            if event.key == pygame.K_BACKSPACE:
                if self.active_input == "rows":
                    self.input_rows = self.input_rows[:-1]
                else:
                    self.input_cols = self.input_cols[:-1]
            elif event.unicode.isdigit():
                if self.active_input == "rows":
                    self.input_rows += event.unicode
                else:
                    self.input_cols += event.unicode
        
        return None

# --- Màn hình 2: Editor ---
class MapEditor:
    def __init__(self, n, m, existing_grid=None, loaded_filepath=None):
        self.n = n
        self.m = m
        self.loaded_filepath = loaded_filepath
        
        # Sidebar chiếm 22% màn hình
        self.sidebar_width_ratio = 0.22
        self.sidebar_width = int(SCREEN_W * self.sidebar_width_ratio)
        
        # Tính toán tile size để vừa khít màn hình
        available_width = int(SCREEN_W * (1 - self.sidebar_width_ratio))
        available_height = SCREEN_H
        tile_w = available_width // m
        tile_h = available_height // n
        self.tile = min(tile_w, tile_h)
        
        # Tính toán offset để căn giữa grid
        self.grid_width = self.tile * m
        self.grid_height = self.tile * n
        self.grid_offset_x = self.sidebar_width + (available_width - self.grid_width) // 2
        self.grid_offset_y = (available_height - self.grid_height) // 2
        
        if existing_grid:
            self.grid = existing_grid
        else:
            self.grid = [[{"type": "empty", "id": None} for _ in range(m)] for _ in range(n)]
        
        self.modes = ["obstacle", "agent", "goal", "erase"]
        self.mode = "obstacle"
        
        # UI elements - sử dụng tọa độ chuẩn hóa (tương đối với sidebar)
        sidebar_w = self.sidebar_width_ratio
        self.tickboxes = {
            text: norm_rect(0.011, 0.05 + i * 0.067, 0.022, 0.033) 
            for i, text in enumerate(self.modes)
        }
        
        self.btn_generate_obs = norm_rect(0.011, 0.35, 0.2, 0.05)
        self.btn_generate_agent = norm_rect(0.011, 0.41, 0.2, 0.05)
        self.btn_generate_goal = norm_rect(0.011, 0.47, 0.2, 0.05)
        self.btn_fill_isolated = norm_rect(0.011, 0.53, 0.2, 0.05)
        self.btn_clear = norm_rect(0.011, 0.59, 0.2, 0.05)
        self.btn_save = norm_rect(0.011, 0.667, 0.2, 0.067)
        self.btn_back = norm_rect(0.011, 0.75, 0.2, 0.05)
        
        self.mouse_pressed = False
    
    def next_available_agent_id(self):
        existing = {c["id"] for row in self.grid for c in row 
                   if c["type"] == "agent" and c["id"] is not None}
        i = 0
        while i in existing:
            i += 1
        return i
    
    def draw_button(self, rect, text, enabled=True):
        color = GREEN if enabled else GRAY
        pygame.draw.rect(screen, color, rect)
        pygame.draw.rect(screen, BLACK, rect, 2)
        label = font.render(text, True, BLACK)
        screen.blit(label, (rect.centerx - label.get_width() // 2, 
                           rect.centery - label.get_height() // 2))
    
    def draw_sidebar(self):
        pygame.draw.rect(screen, (230, 230, 230), (0, 0, self.sidebar_width, SCREEN_H))
        title = font.render("Mode:", True, BLACK)
        screen.blit(title, (int(0.011 * SCREEN_W), int(0.008 * SCREEN_H)))
        
        for text, rect in self.tickboxes.items():
            pygame.draw.rect(screen, WHITE, rect)
            pygame.draw.rect(screen, BLACK, rect, 2)
            if self.mode == text:
                pygame.draw.line(screen, BLACK, rect.topleft, rect.bottomright, 2)
                pygame.draw.line(screen, BLACK, rect.topright, rect.bottomleft, 2)
            label = font.render(text, True, BLACK)
            screen.blit(label, (rect.right + 8, rect.y - 2))
        
        self.draw_button(self.btn_generate_obs, "Generate obstacles")
        self.draw_button(self.btn_generate_agent, "Generate agents")
        any_agent = any(c["type"] == "agent" for row in self.grid for c in row)
        self.draw_button(self.btn_generate_goal, "Generate goals", enabled=any_agent)
        self.draw_button(self.btn_fill_isolated, "Fill isolated areas")
        self.draw_button(self.btn_clear, "Clear map")
        self.draw_button(self.btn_save, "SAVE MAP", enabled=True)
        self.draw_button(self.btn_back, "← Back")
        
        # Help
        help_y = int(0.85 * SCREEN_H)
        help1 = small_font.render("Keys:", True, (50, 50, 50))
        help2 = small_font.render("1=Obs 2=Agent", True, (50, 50, 50))
        help3 = small_font.render("3=Goal 4=Erase", True, (50, 50, 50))
        screen.blit(help1, (int(0.011 * SCREEN_W), help_y))
        screen.blit(help2, (int(0.011 * SCREEN_W), help_y + 25))
        screen.blit(help3, (int(0.011 * SCREEN_W), help_y + 50))
    
    def draw_grid(self):
        for y in range(self.n):
            for x in range(self.m):
                cell = self.grid[y][x]
                cx = self.grid_offset_x + x * self.tile
                cy = self.grid_offset_y + y * self.tile
                rect = pygame.Rect(cx, cy, self.tile, self.tile)
                
                pygame.draw.rect(screen, WHITE, rect)
                pygame.draw.rect(screen, BLACK, rect, 1)
                
                if cell["type"] == "obstacle":
                    pygame.draw.rect(screen, OBSTACLE_COLOR, rect)
                
                elif cell["type"] == "goal":
                    s = int(self.tile * 0.4)
                    gx = cx + (self.tile - s) // 2
                    gy = cy + (self.tile - s) // 2
                    pygame.draw.rect(screen, GOAL_COLOR, (gx, gy, s, s))
                    if cell["id"] is not None:
                        num = small_font.render(str(cell["id"]), True, BLACK)
                        screen.blit(num, (gx + s // 2 - num.get_width() // 2, 
                                         gy + s // 2 - num.get_height() // 2))
                
                elif cell["type"] == "agent":
                    center = (cx + self.tile // 2, cy + self.tile // 2)
                    radius = int(self.tile * 0.4)
                    pygame.draw.circle(screen, AGENT_COLOR, center, radius)
                    pygame.draw.circle(screen, BLACK, center, radius, 1)
                    if cell["id"] is not None:
                        num = small_font.render(str(cell["id"]), True, BLACK)
                        screen.blit(num, (center[0] - num.get_width() // 2, 
                                         center[1] - num.get_height() // 2))
    
    def draw(self):
        screen.fill(WHITE)
        self.draw_sidebar()
        self.draw_grid()
    
    def get_cell(self, pos):
        x, y = pos
        if x < self.grid_offset_x or x >= self.grid_offset_x + self.grid_width:
            return None
        if y < self.grid_offset_y or y >= self.grid_offset_y + self.grid_height:
            return None
        
        gx = (x - self.grid_offset_x) // self.tile
        gy = (y - self.grid_offset_y) // self.tile
        if 0 <= gx < self.m and 0 <= gy < self.n:
            return gy, gx
        return None
    
    def place_tile(self, y, x):
        if self.mode == "erase":
            self.grid[y][x] = {"type": "empty", "id": None}
        elif self.mode == "obstacle":
            self.grid[y][x] = {"type": "obstacle", "id": None}
        elif self.mode == "agent":
            idx = self.next_available_agent_id()
            self.grid[y][x] = {"type": "agent", "id": idx}
        elif self.mode == "goal":
            idx = len([1 for row in self.grid for c in row if c["type"] == "goal"])
            self.grid[y][x] = {"type": "goal", "id": idx}
    
    def generate_obstacles(self, prob=0.15):
        for y in range(self.n):
            for x in range(self.m):
                if self.grid[y][x]["type"] == "empty" and random.random() < prob:
                    self.grid[y][x] = {"type": "obstacle", "id": None}
    
    def generate_agents(self, count=3):
        empty = [(y, x) for y in range(self.n) for x in range(self.m) 
                if self.grid[y][x]["type"] == "empty"]
        random.shuffle(empty)
        
        for _ in range(min(count, len(empty))):
            y, x = empty.pop()
            idx = self.next_available_agent_id()
            self.grid[y][x] = {"type": "agent", "id": idx}
    
    def generate_goals(self):
        goals = set(c["id"] for y in range(self.n) for x, c in enumerate(self.grid[y]) 
                   if c["type"] == "goal")
        
        agents = [(y, x, c["id"]) for y in range(self.n) 
                 for x, c in enumerate(self.grid[y]) 
                 if c["type"] == "agent" and c["id"] not in goals]
        
        empty = [(y, x) for y in range(self.n) for x in range(self.m) 
                if self.grid[y][x]["type"] == "empty"]
        
        random.shuffle(empty)
        for ay, ax, aid in agents:
            if empty:
                y, x = empty.pop()
                self.grid[y][x] = {"type": "goal", "id": aid}
    
    def fill_isolated_areas(self, min_size=10):
        """Tìm và lấp đầy các vùng nhỏ bị cô lập"""
        visited = [[False for _ in range(self.m)] for _ in range(self.n)]
        regions = []
        
        def bfs(start_y, start_x):
            """Tìm tất cả các ô trong vùng liên thông"""
            queue = [(start_y, start_x)]
            region = []
            visited[start_y][start_x] = True
            
            while queue:
                y, x = queue.pop(0)
                region.append((y, x))
                
                # Kiểm tra 4 hướng
                for dy, dx in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                    ny, nx = y + dy, x + dx
                    if (0 <= ny < self.n and 0 <= nx < self.m and 
                        not visited[ny][nx] and 
                        self.grid[ny][nx]["type"] != "obstacle"):
                        visited[ny][nx] = True
                        queue.append((ny, nx))
            
            return region
        
        # Tìm tất cả các vùng liên thông
        for y in range(self.n):
            for x in range(self.m):
                if not visited[y][x] and self.grid[y][x]["type"] != "obstacle":
                    region = bfs(y, x)
                    regions.append(region)
        
        if not regions:
            return
        
        # Tìm vùng lớn nhất
        largest_region = max(regions, key=len)
        
        # Lấp đầy các vùng nhỏ hơn min_size
        filled_count = 0
        for region in regions:
            if len(region) < min_size and region != largest_region:
                for y, x in region:
                    # Chỉ lấp đầy ô empty, giữ nguyên agent và goal
                    if self.grid[y][x]["type"] == "empty":
                        self.grid[y][x] = {"type": "obstacle", "id": None}
                        filled_count += 1
        
        print(f"Filled {filled_count} cells in {len([r for r in regions if len(r) < min_size])} isolated areas")
    
    def save_map(self):
        # Open save file dialog
        initial_dir = os.path.dirname(self.loaded_filepath) if self.loaded_filepath else os.getcwd()
        initial_file = os.path.basename(self.loaded_filepath) if self.loaded_filepath else "map.input.yaml"
        
        filepath = filedialog.asksaveasfilename(
            title="Save map as",
            defaultextension=".yaml",
            filetypes=[
                ("YAML files", "*.yaml"),
                ("All files", "*.*")
            ],
            initialdir=initial_dir,
            initialfile=initial_file
        )
        
        if not filepath:
            return False
        
        agents, goals, obstacles = [], [], []
        for y in range(self.n):
            for x in range(self.m):
                t = self.grid[y][x]["type"]
                if t == "obstacle":
                    obstacles.append((x, self.n - 1 - y))
                elif t == "agent":
                    agents.append({
                        "start": [x, self.n - 1 - y], 
                        "goal": None, 
                        "name": f"agent{self.grid[y][x]['id']}"
                    })
                elif t == "goal":
                    goals.append((x, self.n - 1 - y, self.grid[y][x]["id"]))
        
        for gx, gy, gid in goals:
            for a in agents:
                if a["name"] == f"agent{gid}":
                    a["goal"] = [gx, gy]
                    break
        
        agents.sort(key=lambda e: int(e['name'][5:]))
        
        data = {
            "agents": agents, 
            "map": {"dimensions": [self.n, self.m], "obstacles": obstacles}, 
            "dynamic_obstacles": {}
        }
        
        with open(filepath, "w") as f:
            yaml.dump(data, f, sort_keys=False, default_flow_style=None)
        
        print(f"Map saved to {filepath}")
        self.loaded_filepath = filepath
        return True
    
    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            self.mouse_pressed = True
            pos = event.pos
            
            # Tickbox
            for text, rect in self.tickboxes.items():
                if rect.collidepoint(pos):
                    self.mode = text
            
            # Buttons
            if self.btn_generate_obs.collidepoint(pos):
                self.generate_obstacles()
            elif self.btn_generate_agent.collidepoint(pos):
                self.generate_agents()
            elif self.btn_generate_goal.collidepoint(pos):
                if any(c["type"] == "agent" for row in self.grid for c in row):
                    self.generate_goals()
            elif self.btn_fill_isolated.collidepoint(pos):
                self.fill_isolated_areas()
            elif self.btn_clear.collidepoint(pos):
                self.grid = [[{"type": "empty", "id": None} for _ in range(self.m)] 
                            for _ in range(self.n)]
            elif self.btn_save.collidepoint(pos):
                self.save_map()
            elif self.btn_back.collidepoint(pos):
                return "back"
            
            # Place tile
            cell = self.get_cell(pos)
            if cell:
                y, x = cell
                self.place_tile(y, x)
        
        elif event.type == pygame.MOUSEBUTTONUP:
            self.mouse_pressed = False
        
        elif event.type == pygame.MOUSEMOTION and self.mouse_pressed:
            cell = self.get_cell(event.pos)
            if cell:
                y, x = cell
                self.place_tile(y, x)
        
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_1: self.mode = "obstacle"
            elif event.key == pygame.K_2: self.mode = "agent"
            elif event.key == pygame.K_3: self.mode = "goal"
            elif event.key == pygame.K_4: self.mode = "erase"
        
        return None

# --- Load map từ file ---
def load_map_from_file(filepath):
    def python_tuple(loader, node):
        return tuple(loader.construct_sequence(node))

    yaml.SafeLoader.add_constructor(
        "tag:yaml.org,2002:python/tuple",
        python_tuple
    )
    # Đọc file với SafeLoader để tránh lỗi tuple constructor
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)
    
    dims = data['map']['dimensions']
    n, m = dims[0], dims[1]
    
    grid = [[{"type": "empty", "id": None} for _ in range(m)] for _ in range(n)]
    
    # Load obstacles - xử lý cả list và tuple
    for obstacle in data['map']['obstacles']:
        if isinstance(obstacle, (list, tuple)):
            x, y = obstacle[0], obstacle[1]
        else:
            x, y = obstacle, obstacle
        grid_y = n - 1 - y
        if 0 <= grid_y < n and 0 <= x < m:
            grid[grid_y][x] = {"type": "obstacle", "id": None}
    
    # Load agents and goals
    for agent in data['agents']:
        x, y = agent['start']
        agent_id = int(agent['name'][5:])
        grid_y = n - 1 - y
        if 0 <= grid_y < n and 0 <= x < m:
            grid[grid_y][x] = {"type": "agent", "id": agent_id}
        
        if agent['goal']:
            gx, gy = agent['goal']
            goal_y = n - 1 - gy
            if 0 <= goal_y < n and 0 <= gx < m:
                grid[goal_y][gx] = {"type": "goal", "id": agent_id}
    
    return n, m, grid

# --- Main ---
def main():
    current_screen = "start"
    start_screen = StartScreen()
    editor = None
    
    running = True
    clock = pygame.time.Clock()
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            if current_screen == "start":
                result = start_screen.handle_event(event)
                if result:
                    if result[0] == "new":
                        _, rows, cols = result
                        editor = MapEditor(rows, cols)
                        current_screen = "editor"
                    elif result[0] == "load":
                        _, filepath = result
                        try:
                            n, m, grid = load_map_from_file(filepath)
                            editor = MapEditor(n, m, grid, filepath)
                            current_screen = "editor"
                        except Exception as e:
                            print(f"Error loading file: {e}")
            
            elif current_screen == "editor":
                result = editor.handle_event(event)
                if result == "back":
                    current_screen = "start"
                    start_screen = StartScreen()
        
        if current_screen == "start":
            start_screen.draw()
        elif current_screen == "editor":
            editor.draw()
        
        pygame.display.flip()
        clock.tick(60)
    
    pygame.quit()

if __name__ == "__main__":
    main()