import serial
import time

# Replace 'COMx' with the correct port name for your Arduino
arduino_port = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
arduino_baud_rate = 9600

ser = None  # Global variable to hold the serial connection

def setup_serial_connection():
    global ser, arduino_port, arduino_baud_rate
    ser = serial.Serial(arduino_port, arduino_baud_rate, timeout=1)
    time.sleep(2)  # Initial delay to allow Arduino to reset

def send_steps_to_arduino(delta_steps):
    global ser
    if ser is not None:
        # Combine delta_steps and stepper_delay into a single string, separated by a delimiter
        command = f"{int(delta_steps)},{int(200)}\n"
        ser.write(command.encode())
        response = ser.readline().decode().strip()
        if response:
            print(f"Arduino response: {response}")
        else:
            print("No response from Arduino or timeout occurred.")
    else:
        print("Serial connection not established.")

def close_serial_connection():
    global ser
    if ser is not None:
        ser.close()

def main():
    setup_serial_connection()
    try:
        while True:
            # Your main program logic here
            # Example: send steps to Arduino
            delta_steps = input("Enter the delta steps (signed integer): ")
            if delta_steps.lower() == 'exit':
                break  # Exit the loop to end the program
            if delta_steps.isdigit() or (delta_steps.startswith('-') and delta_steps[1:].isdigit()):
                send_steps_to_arduino(int(delta_steps))
            else:
                print("Please enter a valid signed integer.")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        close_serial_connection()

if __name__ == '__main__':
    main()

