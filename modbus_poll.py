import socket
import struct
from math import ceil

HOST = "127.0.0.1"
PORT = 502

# Modbus function codes
READ_COILS = 0x01
READ_DISCRETE_INPUTS = 0x02
READ_HOLDING_REGISTERS = 0x03
READ_INPUT_REGISTERS = 0x04
WRITE_SINGLE_COIL = 0x05
WRITE_SINGLE_REGISTER = 0x06
WRITE_MULTIPLE_COILS = 0x0F
WRITE_MULTIPLE_REGISTERS = 0x10

# Modbus exception codes
ILLEGAL_FUNCTION = 0x01
ILLEGAL_DATA_ADDRESS = 0x02
ILLEGAL_DATA_VALUE = 0x03
SERVER_DEVICE_FAILURE = 0x04


# Convert decimal (UNIT16) into 2 bytes
def decimal_to_bytes(num):
    byte_1 = (num // 16 ** 2)
    byte_2 = num - byte_1 * 16 ** 2
    return byte_1, byte_2


# Convert 2 bytes into a decimal (UNIT16)
def bytes_to_dec(byte):
    return byte[0] * 16 ** 2 + byte[1]


class ModbusMaster:
    def __init__(self, host=HOST, port=PORT):
        self._host = host
        self._port = port
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.connect((self._host, self._port))

    def _error_display(self, exception_code):
        if exception_code == ILLEGAL_FUNCTION:
            raise Exception("Illegal function")
        elif exception_code == ILLEGAL_DATA_ADDRESS:
            raise Exception("Illegal data address")
        elif exception_code == ILLEGAL_DATA_VALUE:
            raise Exception("Illegal data value")
        elif exception_code == SERVER_DEVICE_FAILURE:
            raise Exception("Server device failure")
        else:
            raise Exception("Unknown exception")

    def _send_packet(self, data):
        self._sock.send(data)

    def _recv_packet(self):
        data = self._sock.recv(1028)
        if not data:
            raise Exception("Connection closed")
        return data

    def _build_packet_reading(self, address, transaction_id, function_code, id, count):
        # TransactionID, Address, ByteCount each requires 2 bytes
        trans_byte_1, trans_byte_2 = decimal_to_bytes(transaction_id)
        addr_byte_1, addr_byte_2 = decimal_to_bytes(address)
        count_byte_1, count_byte_2 = decimal_to_bytes(count)
        packet = struct.pack("12B",
                             trans_byte_1, trans_byte_2,
                             0, 0,  # ProtocolID
                             0, 6,  # Length
                             id,
                             function_code,
                             addr_byte_1, addr_byte_2,
                             count_byte_1, count_byte_2)
        self._send_packet(packet)

    def _build_packet_writing(self, address, transaction_id, function_code, id, values):
        # TransactionID, Address each requires 2 bytes
        trans_byte_1, trans_byte_2 = decimal_to_bytes(transaction_id)
        addr_byte_1, addr_byte_2 = decimal_to_bytes(address)

        if function_code == WRITE_SINGLE_REGISTER:
            # Data needs to be two bytes
            data_byte_1, data_byte_2 = decimal_to_bytes(values[0] if values[0] > -1 else values[0] + 65536)
            packet = struct.pack("12B",
                                 trans_byte_1, trans_byte_2,
                                 0, 0,  # ProtocolID
                                 0, 6,  # Length
                                 id,
                                 function_code,
                                 addr_byte_1, addr_byte_2,
                                 data_byte_1, data_byte_2)

        elif function_code == WRITE_MULTIPLE_REGISTERS:
            # Calculate the length and transform into bytes
            count = len(values)
            count_byte_1, count_byte_2 = decimal_to_bytes(count)

            packet = struct.pack("13B",
                                 trans_byte_1, trans_byte_2,
                                 0, 0,              # ProtocolID
                                 0, 7 + count * 2,  # Length
                                 id,
                                 function_code,
                                 addr_byte_1, addr_byte_2,
                                 count_byte_1, count_byte_2,
                                 count*2)

            # Append each data in form of bytes into the packet
            for value in values:
                data_byte_1, data_byte_2 = decimal_to_bytes(value if value > -1 else value + 65536)
                packet = packet + struct.pack("2B", data_byte_1, data_byte_2)

        elif function_code == WRITE_SINGLE_COIL:
            packet = struct.pack("12B",
                                 trans_byte_1, trans_byte_2,
                                 0, 0,              # ProtocolID
                                 0, 6,              # Length
                                 id,
                                 function_code,
                                 addr_byte_1, addr_byte_2,
                                 values[0] * 255,   # 0xff or 0x00
                                 0)                 # Padding

        else:
            # Calculate the length and transform into bytes
            count = len(values)
            count_byte_1, count_byte_2 = decimal_to_bytes(count)
            count_byte = ceil(count/8)

            packet = struct.pack("13B",
                                 trans_byte_1, trans_byte_2,
                                 0, 0,              # ProtocolID
                                 0, 7 + count_byte, # Length
                                 id,
                                 function_code,
                                 addr_byte_1, addr_byte_2,
                                 count_byte_1, count_byte_2,
                                 count_byte)

            # Convert binaries to BCD:
            # 1010 1000 1 → 1010 1000 1000 0000 → 0101 0001 0001 0000 → 5 1 1 0 → 1501
            value = ''.join(map(str, values)).ljust(count_byte * 8, '0')
            for i in range(0, len(value), 8):
                byte_1 = 0
                byte_2 = 0
                for j in range(4):
                    byte_2 += int(value[i + j]) * (2 ** j)      #
                    byte_1 += int(value[i + j + 4]) * (2 ** j)
                packet = packet + struct.pack("1B", byte_1 * 16 + byte_2)

        self._send_packet(packet)

    def read_discrete_inputs(self, address, transaction_id, count):
        self._build_packet_reading(address, transaction_id, READ_DISCRETE_INPUTS, 1, count)
        response = self._recv_packet()

        if response[1] & 0x80:
            self._error_display(response[2])

        else:
            print('Transaction Identifier:', bytes_to_dec(response[0:2]))
            print('Protocol Identifier:', bytes_to_dec(response[2:4]))
            print('Response length:', bytes_to_dec(response[4:6]))
            print('Unit identifier:', int(response[6]))
            print('Function code:', int(response[7]))
            byte_count = int(response[8])
            print('Byte count:', byte_count)
            coils = []

            for i in range(byte_count):
                value = "{0:b}".format(response[9 + i]).zfill(8)[::-1]
                for i in range(len(value)):
                    if value[i] == '1':
                        coils.append(1)
                    else:
                        coils.append(0)

            for num, coil in enumerate(coils):
                print(f'Register {address + num}:', coil)
                if num == count - 1:
                    break

    def read_coils(self, address, transaction_id, count):
        self._build_packet_reading(address, transaction_id, READ_COILS, 1, count)
        response = self._recv_packet()

        if response[1] & 0x80:
            self._error_display(response[2])
        else:
            print('Transaction Identifier:', bytes_to_dec(response[0:2]))
            print('Protocol Identifier:', bytes_to_dec(response[2:4]))
            print('Response length:', bytes_to_dec(response[4:6]))
            print('Unit identifier:', int(response[6]))
            print('Function code:', int(response[7]))
            byte_count = int(response[8])
            print('Byte count:', byte_count)
            coils = []

            for i in range(byte_count):
                # Zeros fill and reverse bits
                value = "{0:b}".format(response[9 + i]).zfill(8)[::-1]
                for i in range(len(value)):
                    if value[i] == '1':
                        coils.append(1)
                    else:
                        coils.append(0)

            for num, coil in enumerate(coils):
                print(f'Register {address + num}:', coil)
                if num == count - 1:
                    break

    def read_holding_registers(self, address, transaction_id, count):
        self._build_packet_reading(address, transaction_id, READ_HOLDING_REGISTERS, 1, count)
        response = self._recv_packet()

        if response[1] & 0x80:
            self._error_display(response[2])
        else:
            print('Transaction Identifier:', bytes_to_dec(response[0:2]))
            print('Protocol Identifier:', bytes_to_dec(response[2:4]))
            print('Response length:', bytes_to_dec(response[4:6]))
            print('Unit identifier:', int(response[6]))
            print('Function code:', int(response[7]))
            byte_count = int(response[8])
            print('Byte count:', byte_count)
            regs = []

            for i in range(0, byte_count, 2):
                value_1 = int(response[9 + i]) * 16 ** 2
                value_2 = int(response[9 + i + 1])
                value = value_1 + value_2
                regs.append(value if value < 32768 else value - 65536)

            for num, reg in enumerate(regs):
                print(f'Register {address + num}:', reg)
                if num == count - 1:
                    break

    def read_input_registers(self, address, transaction_id, count):
        self._build_packet_reading(address, transaction_id, READ_INPUT_REGISTERS, 1, count)
        response = self._recv_packet()

        if response[1] & 0x80:
            self._error_display(response[2])

        else:
            print('Transaction Identifier:', bytes_to_dec(response[0:2]))
            print('Protocol Identifier:', bytes_to_dec(response[2:4]))
            print('Response length:', bytes_to_dec(response[4:6]))
            print('Unit identifier:', int(response[6]))
            print('Function code:', int(response[7]))
            byte_count = int(response[8])
            print('Byte count:', byte_count)
            regs = []

            for i in range(0, byte_count, 2):
                value_1 = int(response[9 + i]) * 16 ** 2
                value_2 = int(response[9 + i + 1])
                value = value_1 + value_2
                regs.append(value if value < 32768 else value - 65536)

            for num, reg in enumerate(regs):
                print(f'Register {address + num}:', reg)
                if num == count - 1:
                    break

    def write_single_register(self, address, transaction_id, value):
        self._build_packet_writing(address, transaction_id, WRITE_SINGLE_REGISTER, 1, value)
        response = self._recv_packet()

        if response[1] & 0x80:
            self._error_display(response[2])
        else:
            print("Success!")

    def write_multiple_registers(self, address, transaction_id, values):
        self._build_packet_writing(address, transaction_id, WRITE_MULTIPLE_REGISTERS, 1, values)
        response = self._recv_packet()

        if response[1] & 0x80:
            self._error_display(response[2])

        else:
            print("Success!")

    def write_single_coil(self, address, transaction_id, value):
        self._build_packet_writing(address, transaction_id, WRITE_SINGLE_COIL, 1, value)
        response = self._recv_packet()

        if response[1] & 0x80:
            self._error_display(response[2])

        else:
            print("Success!")

    def write_multiple_coils(self, address, transaction_id, values):
        self._build_packet_writing(address, transaction_id, WRITE_MULTIPLE_COILS, 1, values)
        response = self._recv_packet()

        if response[1] & 0x80:
            self._error_display(response[2])

        else:
            print("Success!")


if __name__ == "__main__":
    mm = ModbusMaster()
    transaction_number = 0
    while True:
        print('\nFunction codes:')
        print('---------------')
        print(' Read coils               = 1  |  Write single coil        = 5')
        print(' Read discrete inputs     = 2  |  Write single register    = 6')
        print(' Read holding registers   = 3  |  Write multiple coils     = 7')
        print(' Read input registers     = 4  |  Write multiple registers = 8')
        print(' Exit                     = 0\n')
        function_code = input("Choose function: ")

        # Exit
        if function_code == '0':
            break

        address = int(input("Choose address: "))

        # Read Coils
        if function_code == '1':
            quantity = int(input("Choose quantity of coils: "))
            transaction_number += 1
            mm.read_coils(address, transaction_number, quantity)

        # Read Discrete Inputs
        elif function_code == '2':
            quantity = int(input("Choose quantity of coils: "))
            transaction_number += 1
            mm.read_discrete_inputs(address, transaction_number, quantity)

        # Read Holding Registers
        elif function_code == '3':
            quantity = int(input("Choose quantity of registers: "))
            transaction_number += 1
            mm.read_holding_registers(address, transaction_number, quantity)

        # Read Input Registers
        elif function_code == '4':
            quantity = int(input("Choose quantity of registers: "))
            transaction_number += 1
            mm.read_input_registers(address, transaction_number, quantity)

        # Write Single Coil
        elif function_code == '5':
            value = int(input("Set value 0 or 1: "))
            transaction_number += 1
            mm.write_single_coil(address, transaction_number, [value])

        # Write Single Register
        elif function_code == '6':
            value = int(input("Set value from -32768 to 32767: "))
            transaction_number += 1
            mm.write_single_register(address, transaction_number, [value])

        # Write Multiple Coils
        elif function_code == '7':
            values = input("Set value 0 or 1 separately with SPACE: ").split(' ')

            for i, v in enumerate(values):
                values[i] = int(v)

            transaction_number += 1
            mm.write_multiple_coils(address, transaction_number, values)

        # Write Multiple Registers
        elif function_code == '8':
            values = input("Set value -32768 to 32767 separately with SPACE: ").split(' ')

            for i, v in enumerate(values):
                values[i] = int(v)

            transaction_number += 1
            mm.write_multiple_registers(address, transaction_number, values)

        # Error
        else:
            print('Unknown command!')
