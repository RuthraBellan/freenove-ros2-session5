class Ordinary_Car:
    def __init__(self):
        print("✅ Mock Motor Initialized")
    
    def set_motor_model(self, d1, d2, d3, d4):
        avg_left = (d1 + d2) / 2
        avg_right = (d3 + d4) / 2
        print(f"🚗 Motor: L={avg_left:.0f}, R={avg_right:.0f} | PWM=[{d1},{d2},{d3},{d4}]")
    
    def close(self):
        print("✅ Mock Motor Closed")