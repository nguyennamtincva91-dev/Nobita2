# visualizer.py - Chịu trách nhiệm vẽ hình (Rendering) lên Canvas Tkinter

import tkinter as tk
import math
from conf import SimParam, PhysParam

class CartPoleVisualizer:
    def __init__(self, canvas):
        """
        Khởi tạo Visualizer.
        canvas: Đối tượng tk.Canvas để vẽ lên.
        """
        self.canvas = canvas
        self.width = SimParam.WIN_WIDTH
        self.height = SimParam.WIN_HEIGHT
        
        # Lấy thông số vật lý để biết chiều dài thanh
        self.phys = PhysParam()

        # Tọa độ mặt đất (Vẽ thấp xuống dưới một chút cho đẹp)
        # Tọa độ Y trong Tkinter tính từ trên xuống dưới (0 là đỉnh)
        self.floor_y = self.height / 2 + 100

    def draw(self, state):
        """
        Vẽ lại toàn bộ khung hình dựa trên trạng thái hiện tại.
        state: [theta, theta_dot, x, x_dot]
        """
        # 1. Xóa hình cũ
        self.canvas.delete("all")

        # 2. Giải nén trạng thái
        # Lưu ý: state[0] là theta (góc), state[2] là x (vị trí xe)
        theta = state[0]
        x = state[2]

        # 3. Chuyển đổi tọa độ Thực tế (Mét) -> Tọa độ Màn hình (Pixel)
        cx = self.width / 2  # Tâm màn hình
        scale = SimParam.SCALE

        # --- TÍNH TOÁN TỌA ĐỘ ---
        # Tọa độ Xe (Cart)
        cart_screen_x = cx + x * scale
        cart_screen_y = self.floor_y
        
        # Tọa độ Đầu con lắc (Pole End)
        # Công thức: x_pole = x_cart + L * sin(theta)
        #            y_pole = y_cart - L * cos(theta)
        # Lưu ý: Trục Y màn hình hướng xuống, nên ta phải dùng dấu TRỪ cho cos(theta) 
        # để khi góc = 0 (cos=1), con lắc hướng lên trên.
        pole_len_pixel = self.phys.L * scale
        pole_end_x = cart_screen_x + pole_len_pixel * math.sin(theta)
        pole_end_y = cart_screen_y - pole_len_pixel * math.cos(theta)

        # --- VẼ HÌNH ---
        
        # A. Vẽ Đường ray (Sàn)
        self.canvas.create_line(
            0, self.floor_y, self.width, self.floor_y, 
            fill="gray", width=2
        )

        # B. Vẽ Xe (Cart)
        cart_w = 60 # Chiều rộng xe (pixel)
        cart_h = 30 # Chiều cao xe (pixel)
        self.canvas.create_rectangle(
            cart_screen_x - cart_w/2, cart_screen_y - cart_h/2,
            cart_screen_x + cart_w/2, cart_screen_y + cart_h/2,
            fill=SimParam.COLOR_CART, outline="white", width=2
        )
        
        # Vẽ bánh xe cho sinh động
        wheel_r = 8
        self.canvas.create_oval(cart_screen_x - 20, cart_screen_y + 15, cart_screen_x - 20 + wheel_r*2, cart_screen_y + 15 + wheel_r*2, fill="gray")
        self.canvas.create_oval(cart_screen_x + 5, cart_screen_y + 15, cart_screen_x + 5 + wheel_r*2, cart_screen_y + 15 + wheel_r*2, fill="gray")

        # C. Vẽ Thanh (Pole)
        self.canvas.create_line(
            cart_screen_x, cart_screen_y,
            pole_end_x, pole_end_y,
            fill=SimParam.COLOR_POLE, width=6, capstyle=tk.ROUND
        )

        # D. Vẽ Vật nặng (Mass) - Mô hình Point Mass
        r = 12 # Bán kính vật nặng
        self.canvas.create_oval(
            pole_end_x - r, pole_end_y - r,
            pole_end_x + r, pole_end_y + r,
            fill=SimParam.COLOR_MASS, outline="white", width=2
        )
        
        # E. Hiển thị thông số (HUD)
        #info_text = f"Góc: {theta:.2f} rad\nVị trí: {x:.2f} m"
        #self.canvas.create_text(20, 20, text=info_text, fill="white", anchor="w", font=("Consolas", 12))