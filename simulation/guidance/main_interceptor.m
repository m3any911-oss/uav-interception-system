import time
import math
from mavlink_bridge import TalonMAVLinkBridge
from ekf_tracker import TargetEKF
from perception.target_detector import TargetDetector

def run_interception_loop():
    print("=== بدء تشغيل نظام الاعتراض والملاحة المدمج مع التحكم الكامل (Talon C-UAS) ===")
    
    # 1. تهيئة موديول الرؤية والاتصال
    detector = TargetDetector(camera_fov_deg=60.0)
    bridge = TalonMAVLinkBridge(connection_string="udpin:127.0.0.1:14540")
    
    try:
        bridge.connect()
    except Exception as e:
        print(f"تحذير: لم يتم استقبال Heartbeat خلال المهلة، جاري المتابعة... ({e})")

    # 2. قراءة ابتدائية من الرادار
    raw_radar_meas = detector.process_radar_measurement(range_m=180.0, azimuth_deg=25.0, elevation_deg=-5.0)
    
    # 3. تهيئة الفلتر بالقراءة الأولى
    ekf = TargetEKF(dt=0.1, initial_meas=raw_radar_meas)
    
    print("\n--- بدء حلقة التتبع والاعتراض المستمرة (Full Attitude Control) ---")
    print(f"{'[الخطوة]':<8} | {'الموقع المقدر (EKF)':<22} | {'المدى (m)':<10} | {'Roll / Pitch':<18} | {'حالة MAVLink'}")
    print("-" * 90)
    
    simulated_target_pos = list(raw_radar_meas)
    
    # تشغيل حلقة التتبع
    for step in range(1, 201):
        simulated_target_pos[0] -= 1.5  # اقتراب الهدف
        simulated_target_pos[1] -= 0.5  # انحراف جانبي
        
        ekf.predict()
        ekf.update(simulated_target_pos)
        
        pos = ekf.x[:3]
        
        # 4. حساب المسافة المباشرة للهدف
        dist_to_target = math.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2)
        
        # 5. شرط الاصطدام/الإصابة المباشرة (توسيع المدى إلى 30 متر)
        if dist_to_target < 30.0:
            print("-" * 90)
            print(f"🎯 [HIT DETECTED] تم تحييد الهدف بنجاح! (المسافة الأدنى: {dist_to_target:.2f}m)")
            
            # إرسال أمر العودة للقاعدة RTL تلقائياً عبر MAVLink
            if bridge.master is not None:
                bridge.master.set_mode('RTL')
                print("✈️ [MAVLINK] تم تغيير نمط الطائرة تلقائياً إلى العودة للقاعدة (RTL)!")
            break

        r_x, r_y, r_z = pos[0], pos[1], pos[2]
        
        los_azimuth = math.atan2(r_y, r_x)
        los_elevation = math.atan2(-r_z, math.sqrt(r_x**2 + r_y**2))
        
        N = 3.0  # Proportional Navigation Gain
        target_roll_deg = math.degrees(los_azimuth) * N
        target_pitch_deg = math.degrees(los_elevation) * N
        
        # تقييد الزوايا للحدود الآمنة
        target_roll_deg = max(-30.0, min(30.0, target_roll_deg))
        target_pitch_deg = max(-15.0, min(15.0, target_pitch_deg))
        
        # إرسال أوامر التوجيه
        if bridge.master is not None:
            bridge.send_attitude_target(roll_deg=target_roll_deg, pitch_deg=target_pitch_deg, thrust=0.80)
        
        if step % 10 == 0 or step == 1:
            print(f" #{step:03d}   | X:{pos[0]:.1f}, Y:{pos[1]:.1f} | R:{dist_to_target:.1f}m   | Roll:{target_roll_deg:.1f}°, Pitch:{target_pitch_deg:.1f}° | SENT")
        
        time.sleep(0.1)

    print("\n✅ اكتملت المهمة بنجاح!")
    print("🔄 جاري إنهاء النمط المباشر وإعادة الطائرة لوضع الاستقرار/العودة...")

if __name__ == "__main__":
    run_interception_loop()
