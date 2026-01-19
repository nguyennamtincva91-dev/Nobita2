# plant.py - Phương trình động lực học Vật rắn Tổng quát (General Rigid Body)
# Cập nhật để sử dụng trọng tâm (l_cm) và quán tính (J) từ conf.py

import numpy as np
import math

class CartPoleSystem:
    def __init__(self, params):
        self.p = params # Nhận đối tượng PhysParam từ conf.py

    def get_state_space_matrices(self):
        """
        Trả về ma trận A, B của hệ tuyến tính hóa quanh điểm cân bằng thẳng đứng (Upright).
        Mô hình: General Rigid Body (Vật rắn tổng quát).
        Vector trạng thái: X = [theta, theta_dot, x, x_dot]
        """
        # 1. Lấy thông số từ conf.py (đã được tính gộp)
        M = self.p.M          # Khối lượng xe
        m = self.p.m_total    # Tổng khối lượng con lắc (Thanh + Cầu)
        l = self.p.l_cm       # Khoảng cách từ trục quay đến trọng tâm
        J = self.p.J          # Moment quán tính quay quanh trục quay (Pivot)
        g = self.p.g            
        d = self.p.d          # Hệ số ma sát (nếu có)

        # 2. Tính Mẫu số chung (Determinant của ma trận khối lượng)
        # Hệ phương trình động lực học tuyến tính hóa:
        # (M+m)*x_ddot + m*l*theta_ddot = F - d*x_dot
        # m*l*x_ddot   + J*theta_ddot   = m*g*l*theta
        
        # Giải ra:
        # Det = J*(M+m) - (m*l)^2
        Det = J * (M + m) - (m * l)**2

        # 3. Ma trận A (4x4)
        # Hàng 1: theta_dot = theta_dot (0, 1, 0, 0)
        # Hàng 2: theta_ddot = ...
        #   Hệ số góc theta: m*g*l*(M+m) / Det
        #   Hệ số ma sát (nếu có): m*l*d / Det (nhưng ở đây d thường = 0)
        # Hàng 3: x_dot = x_dot (0, 0, 0, 1)
        # Hàng 4: x_ddot = ...
        #   Hệ số góc theta: - (m*l)^2 * g / Det
        
        # Lưu ý: Cấu trúc biến trạng thái là [theta, theta_dot, x, x_dot]
        A = np.array([
            [0, 1, 0, 0],
            [m * g * l * (M + m) / Det, 0, 0, 0],
            [0, 0, 0, 1],
            [- (m * l)**2 * g / Det, 0, 0, 0]
        ])

        # 4. Ma trận B (4x1)
        # Hàng 2 (theta_ddot): -m*l / Det
        # Hàng 4 (x_ddot):      J / Det
        B = np.array([
            [0],
            [-m * l / Det],
            [0],
            [J / Det]
        ])

        return A, B

    def dynamics(self, state, force):
        """
        Phương trình vi phân phi tuyến (Non-linear Dynamics) để mô phỏng chính xác.
        Giải hệ phương trình Lagrange cho vật rắn.
        Input:
            state: [theta, theta_dot, x, x_dot]
            force: Lực điều khiển u (Newton)
        Output:
            d_state: [theta_dot, theta_ddot, x_dot, x_ddot]
        """
        M = self.p.M
        m = self.p.m_total
        l = self.p.l_cm
        J = self.p.J
        g = self.p.g
        
        theta, theta_dot, x, x_dot = state
        sin_t = math.sin(theta)
        cos_t = math.cos(theta)
        
        # Hệ phương trình Lagrange đầy đủ (Không tuyến tính hóa):
        # 1. (M+m)*x_ddot + m*l*cos(t)*theta_ddot - m*l*sin(t)*theta_dot^2 = F
        # 2. m*l*cos(t)*x_ddot + J*theta_ddot - m*g*l*sin(t) = 0
        
        # Viết lại dưới dạng Ax = b để giải tìm (x_ddot, theta_ddot):
        # [ M+m      m*l*cos ] [x_ddot    ]   [ F + m*l*sin*theta_dot^2 ]
        # [ m*l*cos  J       ] [theta_ddot] = [ m*g*l*sin               ]
        
        # Định thức D (Phụ thuộc vào góc theta)
        D = (M + m) * J - (m * l * cos_t)**2
        
        # Các vế phải (Right hand side)
        rhs_1 = force + m * l * sin_t * (theta_dot**2) # Vế phải pt 1
        rhs_2 = m * g * l * sin_t                      # Vế phải pt 2
        
        # Giải quy tắc Cramer:
        x_ddot = (rhs_1 * J - rhs_2 * m * l * cos_t) / D
        theta_ddot = ((M + m) * rhs_2 - rhs_1 * m * l * cos_t) / D

        # Trả về đạo hàm (Lưu ý thứ tự: theta, theta_dot, x, x_dot)
        return np.array([theta_dot, theta_ddot, x_dot, x_ddot])

    def rk4_step(self, state, force, dt):
        """
        Giải tích phân số Runge-Kutta bậc 4 (RK4) để tăng độ chính xác mô phỏng.
        """
        k1 = self.dynamics(state, force)
        k2 = self.dynamics(state + 0.5 * dt * k1, force)
        k3 = self.dynamics(state + 0.5 * dt * k2, force)
        k4 = self.dynamics(state + dt * k3, force)
        
        new_state = state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
        return new_state