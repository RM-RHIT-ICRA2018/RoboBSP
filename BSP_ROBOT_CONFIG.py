class robot:
    def __init__(self):
        self.mono = 7
        self.UPPER_PID_TYPE = ["SPD", "SPD", "SPD", "SPD", "ANG","ANG", "SPD", "ANG"]
        self.LOWER_PID_TYPE = ["TRQ", "TRQ", "TRQ", "TRQ", "SPD","SPD", "TRQ", "SPD"]
        self.PID_Items_BASIC = ["Chassis_Upper", "Chassis_Lower","Yaw_Upper","Yaw_Lower","Pitch_Upper","Pitch_Lower","Feeding_Upper","Feeding_Lower"]
        self.PID_Items_ADVANCED = ["Chassis_X", "Chassis_Y", "Chassis_Phi", "Yaw_Image", "Pitch_Image"]
        self.PID_Items = self.PID_Items_BASIC + self.PID_Items_ADVANCED

