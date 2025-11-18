class GestureHandler:
    def __init__(self, bot_controller):
        self.bot = bot_controller

    def handle_gesture(self, gesture_name):
        gesture = gesture_name.strip()
        if gesture == "Fist":
            self.bot.get_logger().info(f"Handler: Received '{gesture}' -> Moving Forward")
            self.bot.move_forward()
        elif gesture == "Open_Palm":
            self.bot.get_logger().info(f"Handler: Received '{gesture}' -> Stopping")
            self.bot.stop()
        else:
            self.bot.get_logger().warn(f"Handler: Unknown gesture '{gesture}'")
            self.bot.stop()
