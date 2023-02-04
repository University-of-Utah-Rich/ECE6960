import serial
import serial.tools.list_ports
import os
import imaging
import struct

# get_serial_ports returns a list of available serial ports


def get_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]

# connect_to_serial_port prompts the user to select a serial port to connect to


def connect_to_serial_port():
    ports = get_serial_ports()
    if len(ports) == 0:
        print("There are no available ports to connect to. Is a device connected?")
        exit(1)
    print("Available serial ports:")
    for i, port in enumerate(ports):
        print(f"{i}: {port}")
    port_index = int(
        input("Enter the number of the port you want to connect to: "))
    selected_port = ports[port_index]
    return selected_port

# get_image_directory prompts the user to select a directory to load images from


def get_image_directory():
    image_dir = input("Enter the directory where images are stored: ")
    if not os.path.isdir(image_dir):
        print(f"{image_dir} is not a valid directory.")
        return get_image_directory()
    return image_dir

# compute_checksum computes the checksum of a list of bytes
# The checksum is the sum of all bytes in the list truncated to 16 bits


def compute_checksum(img_bytes):
    checksum = 0
    for b in img_bytes:
        checksum += b
    return checksum & 0xFFFF

# stream_img_to_serial streams an image to a serial port
# The image is converted to a list of bytes and sent to the device
# The format of the data is as follows:
# 4 bytes: 0xFFAA AAFF (start of frame)
# 4 bytes: size of image in little endian format
# size bytes: image bytes in little endian format
# 2 bytes: checksum of image bytes in little endian format
# 4 bytes: 0xAAFF FFAA (end of frame)

def stream_img_to_serial(img, ser):
    # Convert image to a list of bytes
    img_bytes = img.tobytes()
    # Send the number of bytes to the device
    size = len(img_bytes)
    # Send the image bytes to the device in little endian format
    # Note the < in the format string indicates little endian
    b = struct.pack("<HHIpHHH", 0xFFAA, 0xAAFF, size, img_bytes,
                    compute_checksum(img_bytes), 0xAAFF, 0xFFAA)
    ser.write(b)


# main function for the program
# Connects to the serial port and streams images to the device
if __name__ == "__main__":
    # Connect to the serial port
    selected_port = connect_to_serial_port()
    # Open the serial port
    ser = serial.Serial(selected_port)
    # Get the directory where images are stored
    image_dir = get_image_directory()
    print(
        f"Connected to {selected_port} and images will be loaded from {image_dir}")
    # Iterate over the images in the directory
    img_iter = imaging.img_iter(image_dir)
    if len(img_iter) == 0:
        # If there are no images in the directory, exit
        print("There are no images in the directory.")
        exit(1)

    # Iterate over the images and send them to the device
    for img in img_iter:
        img = imaging.resize_image(img)
        img = imaging.scale_intensity(img)
        stream_img_to_serial(img, ser)
    # Close the serial port
    ser.close()
