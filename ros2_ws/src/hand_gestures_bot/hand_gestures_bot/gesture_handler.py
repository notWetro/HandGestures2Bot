class MovementHandler:
    def __init__(self, bot_controller):
        self.bot = bot_controller

    def handle_movement(self, movement_type):
        movement = movement_type.strip().lower()

        if movement == "forward":
            self.bot.get_logger().info("Movement: forward → Moving Forward")
            self.bot.move_forward()

        elif movement in {"back", "backward", "reverse", "rueckwaerts", "ruckwarts", "rückwärts", "rückwaerts", "backwards"}:
            self.bot.get_logger().info("Movement: back → Moving Backward")
            self.bot.move_backward()

        elif movement == "left":
            self.bot.get_logger().info("Movement: left → Turning Left")
            self.bot.turn_left()

        elif movement == "right":
            self.bot.get_logger().info("Movement: right → Turning Right")
            self.bot.turn_right()

        elif movement == "stop":
            self.bot.get_logger().info("Movement: stop → Stopping")
            self.bot.stop()

        else:
            self.bot.get_logger().warn(f"Unknown movement '{movement}'")
            self.bot.stop()
