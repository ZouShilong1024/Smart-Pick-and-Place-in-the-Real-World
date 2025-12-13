#!/home/zz/anaconda3/envs/anygrasp/bin/python3
from pymodbus.client.sync import ModbusTcpClient #pip3 install pymodbus==2.5.3

class HandController():
    def __init__(self):
        self.regdict = {
            'ID': 1000,
            'baudrate': 1001,
            'clearErr': 1004,
            'forceClb': 1009,
            'angleSet': 1486,
            'forceSet': 1498,
            'speedSet': 1522,
            'angleAct': 1546,
            'forceAct': 1582,
            'errCode': 1606,
            'statusCode': 1612,
            'temp': 1618,
            'actionSeq': 2320,
            'actionRun': 2322
        }
        self.ip_address = '192.168.11.210'
        self.port = 6000
        print('打开Modbus TCP连接！')
        self.client = self.open_modbus(self.ip_address, self.port)
        # self.write6('speedSet', [1000, 1000, 1000, 1000, 1000, 1000])
        # self.write6('forceSet', [100, 100, 100, 100, 100, 100])
        
    def open_modbus(self, ip, port):
        client = ModbusTcpClient(ip, port)
        client.connect()
        return client

    def close_connection(self,):
        self.client.close()

    def write_register(self, address, values):
        self.client.write_registers(address, values)

    def read_register(self, address, count):
        response = self.client.read_holding_registers(address, count)
        return response.registers if response.isError() is False else []

    def write6(self, reg_name, val):
        if reg_name in ['angleSet', 'forceSet', 'speedSet']:
            val_reg = []
            for i in range(6):
                val_reg.append(val[i] & 0xFFFF)  # 取低16位
            self.write_register(self.regdict[reg_name], val_reg)
        else:
            print('函数调用错误，正确方式：str的值为\'angleSet\'/\'forceSet\'/\'speedSet\'，val为长度为6的list，值为0~1000，允许使用-1作为占位符')

    def read6(self, reg_name):
        if reg_name in ['angleSet', 'forceSet', 'speedSet', 'angleAct', 'forceAct']:
            val = self.read_register(self.regdict[reg_name], 6)
            if len(val) < 6:
                print('没有读到数据')
                return
            # print('读到的值依次为：', end='')
            # for v in val:
            #     print(v, end=' ')
            # print()
            return val
        
        elif reg_name in ['errCode', 'statusCode', 'temp']:
            val_act = self.read_register(self.regdict[reg_name], 3)
            if len(val_act) < 3:
                print('没有读到数据')
                return
                
            results = []
            
            for i in range(len(val_act)):
                low_byte = val_act[i] & 0xFF            # 低八位
                high_byte = (val_act[i] >> 8) & 0xFF     # 高八位
            
                results.append(low_byte)  # 存储低八位
                results.append(high_byte)  # 存储高八位

            # print('读到的值依次为：', end='')
            # for v in results:
            #     print(v, end=' ')
            # print()
            return results
        
        else:
            print('函数调用错误，正确方式：str的值为\'angleSet\'/\'forceSet\'/\'speedSet\'/\'angleAct\'/\'forceAct\'/\'errCode\'/\'statusCode\'/\'temp\'')

if __name__ == '__main__':
    handcontroller = HandController()
    handcontroller.write6('angleSet', [1000, 1000, 1000, 1000, 1000, 0])