def convert_serial_message(steering, left_speed, right_speed):
    message = f"s{steering}l{left_speed}r{right_speed}\n"
    return message

