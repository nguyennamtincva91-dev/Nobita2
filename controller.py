# controller.py - Tính toán bộ điều khiển LQR (Linear Quadratic Regulator)

import numpy as np
import scipy.linalg

class LQRController:
    def __init__(self, plant):
        self.plant = plant
        self.K = None # Ma trận Gains (sẽ được tính toán)

    def compute_gains(self, Q_diag, R_val):
        """
        Tính toán ma trận K tối ưu dựa trên trọng số Q và R.
        Q_diag: List 4 phần tử [trọng số theta, trọng số d_theta, trọng số x, trọng số d_x]
        R_val: Trọng số pha phạt năng lượng (Scalar)
        """
        # 1. Lấy ma trận hệ thống A, B
        A, B = self.plant.get_state_space_matrices()

        # 2. Tạo ma trận trọng số Q và R
        # Q: Ma trận phạt sai lệch trạng thái (4x4)
        Q = np.diag(Q_diag)
        
        # R: Ma trận phạt tín hiệu điều khiển (1x1)
        R = np.array([[R_val]])

        # 3. Giải phương trình Riccati đại số liên tục (CARE)
        # A.T * P + P * A - P * B * R^-1 * B.T * P + Q = 0
        P = scipy.linalg.solve_continuous_are(A, B, Q, R)

        # 4. Tính ma trận Gain K
        # K = R^-1 * B.T * P
        # Vì R là scalar 1x1, R^-1 = 1/R
        self.K = np.dot(scipy.linalg.inv(R), np.dot(B.T, P))
        
        print("--- LQR Computed ---")
        print(f"Q weights: {Q_diag}")
        print(f"R weight: {R_val}")
        print(f"Feedback Gains K: {self.K}")
        
        return self.K

    def get_action(self, state):
        """
        Tính lực điều khiển u = -K * (State - Target)
        State hiện tại: [theta, theta_dot, x, x_dot]
        """
        if self.K is None:
            return 0.0
            
        # Trạng thái mục tiêu (tất cả bằng 0: đứng thẳng, giữa màn hình)
        target = self.plant.p.target_state 
        
        # Sai lệch
        error = state - target
        
        # Luật điều khiển phản hồi trạng thái: u = -K * error
        # Kết quả là ma trận 1x1, lấy ra giá trị scalar
        u = -np.dot(self.K, error).item()
        
        return u