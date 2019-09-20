import os, math, argparse,sys
from periphery import I2C
from periphery import GPIO
from time import sleep

BLOCK_SIZE = 64 # 128 for parts with >8k flash

REQ_ENTER = (0xde, 0xad, 0xbe, 0xef)
ACK  = (0xaa, 0xbb)
NACK = (0xde, 0xad)
FILE = None

def crc8_update(data, crc):
    crc ^= data
    for i in range(0, 8):
        if crc & 0x80 != 0:
            crc = (crc << 1) ^ 0x07
        else:
            crc <<= 1
    return crc & 0xFF

def get_crc():
    crc = 0
    data = open(FILE, 'rb')
    with data as f:
        chunk = bytearray(f.read(BLOCK_SIZE))
        while chunk:
            chunk.extend([0xFF] * (BLOCK_SIZE - len(chunk)))
            for i in chunk:
                crc = crc8_update(i, crc)
            chunk = bytearray(f.read(BLOCK_SIZE))
    return crc

def bootloader_enter(ser, address, rst_gpio,inverted):
    
    # send payload
    req = bytearray(REQ_ENTER)
    chunks = os.path.getsize(FILE)
    chunks = int(math.ceil(float(chunks) / BLOCK_SIZE))
    print('Need to send %s chunks' % chunks)
    crc = get_crc()
    req.extend([chunks, crc, crc])
    # I2c transaction with bootloader request and ACK nand respond
    # toggle reset via sysFS GPIO
    rst_gpio.write(inverted)
    sleep(0.5)
    rst_gpio.write(not inverted)
    sleep(0.01)
    snd=[I2C.Message(req)]
    rcv=[I2C.Message([0x00,0x00], read=True)] 
    ser.transfer(address,snd)
    sleep(0.01) # give some time to process the block
    ser.transfer(address,rcv)
    if bytearray(rcv[0].data) != bytearray(ACK):
        print("Bootloader NACK")
        sys.exit(1)

    return ser

def bootloader_exec(port, address, rst , inverted):
    ser = I2C(port)
    rst_gpio = GPIO(rst, "out")
    bootloader_enter(ser,address,rst_gpio,inverted)
    rst_gpio.close()
    data = open(FILE, 'rb')
    total = 0
    idx = 0
    with data as f:
        chunk = bytearray(f.read(BLOCK_SIZE))
        while chunk:
            chunk.extend([0xFF] * (BLOCK_SIZE - len(chunk)))
            total += len(chunk)
            idx +=1
            sys.stdout.write(str(idx))
            sys.stdout.write("[")
            sys.stdout.write(str(total))
            ser.transfer(address,[I2C.Message(chunk)])
            print("] OK")
            chunk = bytearray(f.read(BLOCK_SIZE))
            sleep(0.1) # I2c could be to fast , give some time to process the block

        sleep(0.05) # I2c could be to fast , give some time to process the block
        ack=[I2C.Message([0x00,0x00],read=True)]
        ser.transfer(address,ack)
        rack=bytearray(ack[0].data)
        if rack == bytearray(ACK):
            print('Done')
        elif rack == bytearray(NACK):
            print('CRC mismatch')
        else:
            print('Invalid response')
    ser.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='stm8-bootloader update utility I2C edition')
    parser.add_argument('--port', '-p', default='/dev/i2c-1')
    parser.add_argument('--address','-a', default=0x22)
    parser.add_argument('--rst', '-r', default=4)
    parser.add_argument('--inverted','-i', default=False, action="store_true")
    parser.add_argument('file', help='firmware in binary format')
    args = parser.parse_args()
    FILE = args.file
    bootloader_exec(args.port, args.address, args.rst, args.inverted)