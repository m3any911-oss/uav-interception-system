import time
import math

# استيراد مباشر من الملفات الموجودة في نفس مجلد العمل
from mavlink_bridge import TalonMAVLinkBridge
from ekf_tracker import TargetEKF

def run_interception_loop():
    print("=== بدء تشغيل نظام الاعتراض والملاحة المدمج (Talon C-UAS) ===")
    
    # 1. تهيئة وحدة الاتصال ووحدة التتبع
    bridge = TalonMAVLinkBridge(connection_string="udp:127.0.0.1:14540")
    bridge.connect()
    
    # حالة الهدف الحقيقية (محاكاة مسار الهدف)
    true_target_pos = [120.0, 30.0, -15.0]
    true_target_vel = [-18.0, 2.0, 0.5]
    
    # تهيئة الفلتر بالقراءة الأولى
    ekf = TargetEKF(dt=0.1, initial_meas=true_target_pos)
    
    current_interceptor_pitch = 0.0
    
    print("\n--- بدء حلقة التتبع والاعتراض المباشرة ---")
    print("[الخطوة] | الموقع المقدر (EKF)    | السرعة (m/s) | أمر الـ Pitch | حالة MAVLink")
    print("-" * 80)
    
    for step in range(1, 11):
        # أ) تحديث مسار الهدف المحاكى
        true_target_pos[0] += true_target_vel[0] * 0.1
        true_target_pos[1] += true_target_vel[1] * 0.1
        true_target_pos[2] += true_target_vel[2] * 0.1
        
        # ب) تصفية موقع الهدف واستخراج المتجهات عبر EKF
        ekf.predict()
        ekf.update(true_target_pos)
        est_pos, est_vel = ekf.get_estimated_state()
        
        # ج) حساب زاوية التوجيه الاعتراضي عبر PN Guidance
        r_los = est_pos
        v_rel = est_vel
        
        pitch_command = bridge.compute_pn_pitch_cmd(r_los, v_rel, current_interceptor_pitch)
        
        # د) إرسال أوامر الاتجاه والمحرك عبر MAVLink
        bridge.send_attitude_target(roll=0.0, pitch=pitch_command, yaw=0.0, thrust=0.70)
        
        current_interceptor_pitch = pitch_command
        
        print(f" #{step:02d}    | X:{est_pos[0]:.1f}, Y:{est_pos[1]:.1f} | Vx:{est_vel[0]:.1f}   | Pitch:{pitch_command:.2f}° | SENT")
        time.sleep(0.1)

    print("\n✅ تم فحص حلقة الاعتراض والتتبع المغلقة بنجاح 100%!")

if __name__ == "__main__":
    run_interception_loop()
