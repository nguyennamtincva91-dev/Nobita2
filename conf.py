# conf.py - Cập nhật tính toán Vật rắn (Rigid Body)

import numpy as np

class PhysParam:
    def __init__(self):
        # --- 1. THÔNG SỐ CƠ BẢN (NHẬP VÀO) ---
        self.M = 0.5        # Khối lượng xe (kg)
        self.L = 0.3        # Chiều dài thanh (m)
        self.g = 9.81       # Gia tốc trọng trường
        self.d = 0.0        # Ma sát

        # --- 2. CẤU TRÚC VẬT RẮN (Thanh + Cầu) ---
        # Theo yêu cầu: Tổng khối lượng là 200g (0.2kg)
        # Bạn có thể chia tỷ lệ tùy ý. Ví dụ: Thanh 100g, Cầu 100g.
        self.m_pole = 0.1   # Khối lượng riêng của thanh (kg)
        self.m_ball = 0.1   # Khối lượng riêng của quả cầu (kg)
        
        # --- 3. TỰ ĐỘNG TÍNH TOÁN (DERIVED PARAMETERS) ---
        # Tổng khối lượng con lắc
        self.m_total = self.m_pole + self.m_ball 

        # A. Tính vị trí Trọng tâm (Center of Mass - l_cm) tính từ trục quay
        # Công thức moment: (m_pole * r_pole + m_ball * r_ball) / m_total
        # - Trọng tâm thanh nằm giữa: L/2
        # - Trọng tâm cầu nằm ở đỉnh: L
        self.l_cm = (self.m_pole * (self.L / 2) + self.m_ball * self.L) / self.m_total

        # B. Tính Moment quán tính quay quanh trục (Moment of Inertia - J_pivot)
        # - I_pole (quanh đầu thanh) = 1/3 * m * L^2
        # - I_ball (chất điểm ở đầu) = m * L^2
        J_pole = (1/3) * self.m_pole * (self.L**2)
        J_ball = self.m_ball * (self.L**2)
        self.J = J_pole + J_ball

        # In ra để kiểm tra
        print(f"--- CẤU TRÚC VẬT LÝ MỚI ---")
        print(f"Tổng khối lượng m: {self.m_total:.3f} kg")
        print(f"Trọng tâm l_cm: {self.l_cm:.4f} m (Lệch về phía đỉnh)")
        print(f"Quán tính J: {self.J:.6f}")

        # Target state
        # Trạng thái mục tiêu (Cân bằng thẳng đứng, xe ở giữa)
        # Vector trạng thái: [theta, theta_dot, x, x_dot]
        self.target_state = np.array([0.0, 0.0, 0.0, 0.0])
        
        self.dt = 0.02 # Bước thời gian lấy mẫu (20ms = 50Hz)

class SimParam:
    COLOR_BG = "#570080"
    COLOR_CART = "#00CED1"
    COLOR_POLE = "#FFFFFF"
    COLOR_MASS = "#FF4500"
    SCALE = 800 
    WIN_WIDTH = 1000
    WIN_HEIGHT = 700