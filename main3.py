import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import collections
import numpy as np

from conf import PhysParam, SimParam
from plant import CartPoleSystem
from controller import LQRController
from visualizer import CartPoleVisualizer

class MainApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Đồ án Cơ Điện Tử: Con Lắc Ngược Cart-Pole")
        
        # 1. Cấu hình cửa sổ rộng
        self.side_panel_width = 600
        self.root.geometry(f"{SimParam.WIN_WIDTH + self.side_panel_width}x{SimParam.WIN_HEIGHT + 30}")

        # --- KHỞI TẠO HỆ THỐNG ---
        self.phys_param = PhysParam()
        self.plant = CartPoleSystem(self.phys_param)
        self.controller = LQRController(self.plant)
        
        # Tính LQR lần đầu
        self.recalc_lqr()

        # Biến trạng thái
        self.running = False
        self.state = np.array([0.1, 0.0, 0.0, 0.0]) # [theta, theta_dot, x, x_dot]
        self.time = 0.0
        self.manual_force = 0.0 

        # Cấu hình đồ thị mượt (300 điểm)
        self.history_len = 300 
        self.t_data = collections.deque(maxlen=self.history_len)
        self.theta_data = collections.deque(maxlen=self.history_len)
        self.dtheta_data = collections.deque(maxlen=self.history_len)
        self.x_data = collections.deque(maxlen=self.history_len)
        self.dx_data = collections.deque(maxlen=self.history_len)

        self.setup_ui()

    def recalc_lqr(self):
        # Tính lại bộ điều khiển
        Q = [100.0, 1.0, 10.0, 1.0]
        R = 0.1
        self.controller.compute_gains(Q, R)

    def apply_force(self, force):
        # Hàm nhận lực từ nút bấm
        self.manual_force = force

    def setup_ui(self):
        main_pane = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_pane.pack(fill=tk.BOTH, expand=True)

        # === KHUNG TRÁI: MÔ PHỎNG ===
        left_frame = ttk.Frame(main_pane)
        main_pane.add(left_frame, weight=2)
        
        self.canvas = tk.Canvas(left_frame, bg=SimParam.COLOR_BG, width=SimParam.WIN_WIDTH, height=SimParam.WIN_HEIGHT)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.viz = CartPoleVisualizer(self.canvas)
        
        # Ép visualizer dùng chung bộ tham số với MainApp ngay từ đầu
        if hasattr(self.viz, 'phys'):
            self.viz.phys = self.phys_param

        self.hud_label = tk.Label(left_frame, text="SẴN SÀNG", bg="black", fg="#00ff00", font=("Consolas", 11), justify=tk.LEFT)
        self.hud_label.place(x=10, y=10)

        # === KHUNG PHẢI: BẢNG ĐIỀU KHIỂN ===
        right_frame = ttk.Frame(main_pane, width=self.side_panel_width)
        main_pane.add(right_frame, weight=3)

        self.notebook = ttk.Notebook(right_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        # Tab 1: Vận hành
        tab_dashboard = ttk.Frame(self.notebook)
        self.notebook.add(tab_dashboard, text="Vận hành & UART")
        self.setup_dashboard_tab(tab_dashboard)

        # Tab 2: Cấu hình
        tab_config = ttk.Frame(self.notebook)
        self.notebook.add(tab_config, text="Cấu hình Hệ thống")
        self.setup_config_tab(tab_config)

    def setup_dashboard_tab(self, parent):
        # 1. UART Mockup
        uart_grp = ttk.LabelFrame(parent, text="UART Mô phỏng", padding=5)
        uart_grp.pack(fill=tk.X, padx=5, pady=5)
        
        f2 = ttk.Frame(uart_grp); f2.pack(fill=tk.X, pady=2)
        ttk.Label(f2, text="Gói tin [Góc, Xe]:").pack(side=tk.LEFT)
        self.entry_mock = ttk.Entry(f2, width=20)
        self.entry_mock.insert(0, "0.2, -0.3")
        self.entry_mock.pack(side=tk.LEFT, padx=5)
        ttk.Button(f2, text="NẠP DỮ LIỆU", command=self.mock_uart_send).pack(side=tk.LEFT)

        # 2. Simulation Control
        sim_grp = ttk.LabelFrame(parent, text="Điều khiển Mô phỏng", padding=5)
        sim_grp.pack(fill=tk.X, padx=5, pady=5)
        
        btn_frame = ttk.Frame(sim_grp)
        btn_frame.pack(fill=tk.X)
        ttk.Button(btn_frame, text="CHẠY", command=self.start_sim).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(btn_frame, text="DỪNG", command=self.stop_sim).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(btn_frame, text="RESET", command=self.reset_sim).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # [FIX 1] 3. Tác động ngoại lực (ĐÃ THÊM LẠI)
        force_grp = ttk.LabelFrame(parent, text="Tác động Ngoại lực", padding=5)
        force_grp.pack(fill=tk.X, padx=5, pady=5)

        # Nút Đẩy Trái
        btn_left = tk.Button(force_grp, text="<< ĐẨY TRÁI", bg="#ffcccc", height=2, font=("Arial", 10, "bold"))
        btn_left.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        btn_left.bind('<ButtonPress-1>', lambda e: self.apply_force(-15))
        btn_left.bind('<ButtonRelease-1>', lambda e: self.apply_force(0))

        # Nút Đẩy Phải
        btn_right = tk.Button(force_grp, text="ĐẨY PHẢI >>", bg="#ccffcc", height=2, font=("Arial", 10, "bold"))
        btn_right.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        btn_right.bind('<ButtonPress-1>', lambda e: self.apply_force(15))
        btn_right.bind('<ButtonRelease-1>', lambda e: self.apply_force(0))

        # 4. Đồ thị
        graph_grp = ttk.LabelFrame(parent, text="Thông số thời gian thực", padding=5)
        graph_grp.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(6, 4.5))
        self.fig.tight_layout(pad=2.0)

        self.lines = {}
        self.lines['theta'], = self.ax1.plot([], [], 'r', lw=1.5); self.ax1.set_title("Góc (Theta)", fontsize=9, fontweight='bold'); self.ax1.grid(True)
        self.lines['dtheta'], = self.ax2.plot([], [], 'orange', lw=1.5); self.ax2.set_title("Vận tốc Góc", fontsize=9, fontweight='bold'); self.ax2.grid(True)
        self.lines['x'], = self.ax3.plot([], [], 'b', lw=1.5); self.ax3.set_title("Vị trí Xe (x)", fontsize=9, fontweight='bold'); self.ax3.grid(True)
        self.lines['dx'], = self.ax4.plot([], [], 'c', lw=1.5); self.ax4.set_title("Vận tốc Xe", fontsize=9, fontweight='bold'); self.ax4.grid(True)

        self.canvas_agg = FigureCanvasTkAgg(self.fig, master=graph_grp)
        self.canvas_agg.draw()
        self.canvas_agg.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def setup_config_tab(self, parent):
        lbl_intro = ttk.Label(parent, text="Thay đổi thông số & Quan sát hình ảnh thay đổi ngay lập tức", font=("Arial", 10))
        lbl_intro.pack(pady=10)

        self.entries = {}
        form_frame = ttk.Frame(parent)
        form_frame.pack(padx=20, pady=5, fill=tk.X)

        params = [
            ("Khối lượng Xe (M) [kg]", "M"),
            ("KL Thanh (m_pole) [kg]", "m_pole"),
            ("KL Cầu (m_ball) [kg]", "m_ball"),
            ("Chiều dài Thanh (L) [m]", "L"),
            ("Trọng trường (g)", "g")
        ]

        for i, (label_text, param_name) in enumerate(params):
            lbl = ttk.Label(form_frame, text=label_text, font=("Arial", 10))
            lbl.grid(row=i, column=0, sticky=tk.W, pady=8)
            ent = ttk.Entry(form_frame, font=("Arial", 10))
            val = getattr(self.phys_param, param_name)
            ent.insert(0, str(val))
            ent.grid(row=i, column=1, sticky=tk.E, pady=8, padx=10)
            self.entries[param_name] = ent

        btn_update = ttk.Button(parent, text="CẬP NHẬT NGAY LẬP TỨC", command=self.update_params)
        btn_update.pack(pady=20, fill=tk.X, padx=40)
        
        lbl_info = ttk.Label(parent, text="Lưu ý: Hình dạng con lắc và Bộ điều khiển sẽ thay đổi theo.", foreground="gray")
        lbl_info.pack()

    def update_params(self):
        try:
            # 1. Lấy dữ liệu từ GUI
            self.phys_param.M = float(self.entries["M"].get())
            self.phys_param.m_pole = float(self.entries["m_pole"].get())
            self.phys_param.m_ball = float(self.entries["m_ball"].get())
            self.phys_param.L = float(self.entries["L"].get())
            self.phys_param.g = float(self.entries["g"].get())
            
            # 2. Tính lại thông số dẫn xuất
            p = self.phys_param
            p.m_total = p.m_pole + p.m_ball
            p.l_cm = (p.m_pole * (p.L / 2) + p.m_ball * p.L) / p.m_total
            J_pole = (1/3) * p.m_pole * (p.L**2)
            J_ball = p.m_ball * (p.L**2)
            p.J = J_pole + J_ball
            
            # 3. Tính lại LQR
            self.recalc_lqr()
            
            # [FIX 2] 4. ĐỒNG BỘ HÓA VỚI VISUALIZER
            # Gán đè bộ tham số mới vào visualizer để nó vẽ đúng chiều dài mới
            if hasattr(self.viz, 'phys'):
                self.viz.phys = self.phys_param
            
            # Vẽ lại ngay lập tức để thấy sự thay đổi
            self.viz.draw(self.state)
            
            messagebox.showinfo("Cập nhật thành công", 
                              f"Đã thay đổi hình dáng và vật lý!\nChiều dài mới: {p.L}m")
            
        except ValueError:
            messagebox.showerror("Lỗi nhập liệu", "Vui lòng chỉ nhập số thực.")

    def start_sim(self):
        if not self.running:
            self.running = True
            self.loop()

    def stop_sim(self):
        self.running = False

    def reset_sim(self):
        self.stop_sim()
        self.state = np.array([0.1, 0.0, 0.0, 0.0])
        self.time = 0
        self.t_data.clear(); self.theta_data.clear(); self.dtheta_data.clear()
        self.x_data.clear(); self.dx_data.clear()
        self.viz.draw(self.state)
        self.canvas_agg.draw()
        self.update_hud()

    def loop(self):
        if not self.running:
            return

        dt = self.phys_param.dt
        u = self.controller.get_action(self.state) + self.manual_force
        self.state = self.plant.rk4_step(self.state, u, dt)
        self.time += dt

        self.update_gui_components()
        self.root.after(int(dt*1000), self.loop)

    def update_gui_components(self):
        self.viz.draw(self.state)
        self.update_hud()

        # Update Graph 30 FPS (0.033s)
        if len(self.t_data) == 0 or (self.time - self.t_data[-1] >= 0.033): 
            self.t_data.append(self.time)
            self.theta_data.append(self.state[0])
            self.dtheta_data.append(self.state[1])
            self.x_data.append(self.state[2])
            self.dx_data.append(self.state[3])
            
            self.lines['theta'].set_data(self.t_data, self.theta_data)
            self.lines['dtheta'].set_data(self.t_data, self.dtheta_data)
            self.lines['x'].set_data(self.t_data, self.x_data)
            self.lines['dx'].set_data(self.t_data, self.dx_data)
            
            for ax in [self.ax1, self.ax2, self.ax3, self.ax4]:
                ax.relim(); ax.autoscale_view()
            
            self.canvas_agg.draw_idle()

    def update_hud(self):
        t, dt, x, dx = self.state
        txt = f"THÔNG SỐ THỜI GIAN THỰC:\nTheta: {t:.4f} rad | dTheta: {dt:.4f} rad/s\nPos X: {x:.4f} m   | Vel dX: {dx:.4f} m/s"
        self.hud_label.config(text=txt)

    def mock_uart_send(self):
        try:
            val = [float(v) for v in self.entry_mock.get().split(',')]
            self.state = np.array([val[0], 0.0, val[1], 0.0])
            self.stop_sim()
            self.update_gui_components()
        except:
            messagebox.showerror("Lỗi", "Nhập sai định dạng!")

if __name__ == "__main__":
    root = tk.Tk()
    app = MainApp(root)
    root.mainloop()