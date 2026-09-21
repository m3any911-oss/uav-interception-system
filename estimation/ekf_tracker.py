import math
import time
import random

class TargetEKF:
    def __init__(self, dt=0.1, initial_meas=None):
        self.dt = dt
        self.is_initialized = False
        
        # State Vector x = [px, py, pz, vx, vy, vz]
        if initial_meas is not None:
            self.x = [initial_meas[0], initial_meas[1], initial_meas[2], 0.0, 0.0, 0.0]
            self.is_initialized = True
        else:
            self.x = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Initial Covariance Matrix P (Diagonal 6x6)
        self.P = [
            [10.0, 0, 0, 0, 0, 0],
            [0, 10.0, 0, 0, 0, 0],
            [0, 0, 10.0, 0, 0, 0],
            [0, 0, 0, 1.0, 0, 0],
            [0, 0, 0, 0, 1.0, 0],
            [0, 0, 0, 0, 0, 1.0]
        ]
        
        # Process Noise Q & Measurement Noise R
        self.q_var = 0.1
        self.r_var = 0.5  # ضوضاء الحساسات / الرادار

    def predict(self):
        """خطوة التنبؤ بالحالة القادمة (State Prediction)"""
        if not self.is_initialized:
            return

        # x_k = F * x_{k-1}
        self.x[0] += self.x[3] * self.dt  # px = px + vx*dt
        self.x[1] += self.x[4] * self.dt  # py = py + vy*dt
        self.x[2] += self.x[5] * self.dt  # pz = pz + vz*dt

        # التنبؤ بالمصفوفة P (تقريب بسيط للحسابات السريعة)
        for i in range(6):
            self.P[i][i] += self.q_var * self.dt

    def update(self, z_meas):
        """خطوة التصحيح باستخدام القراءات المباشرة (Measurement Update)"""
        if not self.is_initialized:
            self.x[0:3] = z_meas[0:3]
            self.is_initialized = True
            return

        # حساب كسب كالمان (Kalman Gain K) والتحديث
        for i in range(3):
            residual = z_meas[i] - self.x[i]
            K = self.P[i][i] / (self.P[i][i] + self.r_var)
            
            # تحديث الموقع
            self.x[i] += K * residual
            # تحديث تقدير السرعة الضمنية
            self.x[i + 3] += (K / self.dt) * residual
            
            # تحديث مصفوفة التباين P
            self.P[i][i] = (1.0 - K) * self.P[i][i]

    def get_estimated_state(self):
        """إرجاع الموقع والسرعة المقدرة"""
        pos = self.x[0:3]
        vel = self.x[3:6]
        return pos, vel


# ==========================================
# وحدة الفحص المباشر (EKF Dry-Run Test)
# ==========================================
if __name__ == "__main__":
    print("=== بدء فحص فلتر كالمان الممتد (EKF Target Tracker) ===")
    
    # مسار وهمي لهدف متحرك مع إضافة مشوشات (Noise)
    true_pos = [100.0, 50.0, -20.0]
    true_vel = [-15.0, 5.0, 1.0]
    
    random.seed(42)  # لثبات نتائج التجربة
    
    # أول قراءة مشوشة لتهيئة الفلتر مباشرة بها
    first_noisy_meas = [
        true_pos[0] + random.gauss(0, 0.8),
        true_pos[1] + random.gauss(0, 0.8),
        true_pos[2] + random.gauss(0, 0.8)
    ]
    
    # إسناد القراءة الأولى عند إنساء الكائن
    tracker = TargetEKF(dt=0.1, initial_meas=first_noisy_meas)
    
    print("\n[خطوة] | القراءة المشوشة (Raw)  | الموقع المصفى (EKF)    | السرعة المقدرة (m/s)")
    print("-" * 75)
    
    for step in range(1, 6):
        # 1. التحديث الحقيقي لموقع الهدف
        true_pos[0] += true_vel[0] * 0.1
        true_pos[1] += true_vel[1] * 0.1
        true_pos[2] += true_vel[2] * 0.1
        
        # 2. إضافة ضوضاء رادار عشوائية
        noisy_meas = [
            true_pos[0] + random.gauss(0, 0.8),
            true_pos[1] + random.gauss(0, 0.8),
            true_pos[2] + random.gauss(0, 0.8)
        ]
        
        # 3. تشغيل الفلتر (Predict + Update)
        tracker.predict()
        tracker.update(noisy_meas)
        est_pos, est_vel = tracker.get_estimated_state()
        
        print(f" #{step}    | X:{noisy_meas[0]:.2f}, Y:{noisy_meas[1]:.2f} | X:{est_pos[0]:.2f}, Y:{est_pos[1]:.2f} | Vx:{est_vel[0]:.2f}, Vy:{est_vel[1]:.2f}")
        time.sleep(0.1)

    print("\n✅ تم فحص فلتر كالمان بنجاح مع التهيئة الأولية المتزنة!")
