# 20210124 Sub側1号車のみ
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import RPi.GPIO as GPIO
import struct, binascii, serial
import time
import datetime


GPIO_Stanby = 17
Train = [0] * 5

StSpFG1 = 0
StSpFG2 = 0
StSpFG3 = 0
StSpFG4 = 0

StartSub1 = 0
StartSub2 = 0
StartSub3 = 0
StartSub4 = 0
StartMain1 = 0
StartMain2 = 0
StartMain3 = 0
StartMain4 = 0

TimeSetSub1 = 0
TimeSetSub2 = 0
TimeSetSub3 = 0
TimeSetSub4 = 0
TimeSetMain1 = 0
TimeSetMain2 = 0
TimeSetMain3 = 0
TimeSetMain4 = 0

iSub = 0
iMain = 0

FG_STRT = 0
FG13_21 = 0
FG22_32 = 0

TT1=[ 10, 10,  30,   5,  10,   4,  20,  20,  20]
TT2=[  5, 15,   6,  20,   5,  15,  30,  40,  10]

GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_Stanby, GPIO.IN)
stick = serial.Serial("/dev/ttyUSB0", 115200)
serMEGA = serial.Serial('/dev/ttyACM0', 2000000, timeout = 1)
serMEGA.flush()
time.sleep(1)

if __name__ == '__main__':
    print('start waiting')
    nowTime = datetime.datetime.now()             # <K7>
    dtINIT = nowTime + datetime.timedelta(days=1) # <K7>
    serMEGA.write(b"I\n")
    JG = GPIO.input(GPIO_Stanby)
    while not JG:
        JG = GPIO.input(GPIO_Stanby)
    serMEGA.write(b"G\n")
    time.sleep(1)


def DispatchSub():
    global StartSub1
    global StartSub2
    global StartSub3
    global StartSub4
    global iSub
    global FG22_32
    
    if(Train[0]=="22" and StartSub1==0 and FG22_32==0):
        now = datetime.datetime.now()
        TimeSetSub1 = now + datetime.timedelta(seconds=TT1[iSub])
        StartSub1 = 1
        iSub = iSub + 1
        return TimeSetSub1
    
    if(Train[1]=="22" and StartSub2==0):
        now = datetime.datetime.now()
        TimeSetSub2 = now + datetime.timedelta(seconds=TT1[iSub])
        StartSub2 = 1
        iSub = iSub + 1
        return TimeSetSub2
    
    if(Train[2]=="22" and StartSub3==0):
        now = datetime.datetime.now()
        TimeSetSub3 = now + datetime.timedelta(seconds=TT1[iSub])
        StartSub3 = 1
        iSub = iSub + 1
        return TimeSetSub3
    
    if(Train[3]=="22" and StartSub4==0):
        now = datetime.datetime.now()
        TimeSetSub4 = now + datetime.timedelta(seconds=TT1[iSub])
        StartSub4 = 1
        iSub = iSub + 1
        return TimeSetSub4

def DispatchMain():
    global StartMain1
    global StartMain2
    global StartMain3
    global StartMain4
    global iMain
    
    if(Train[0]=="32" and StartMain1==0):
        now = datetime.datetime.now()
        TimeSetMain1 = now + datetime.timedelta(seconds=TT2[iMain])
        StartMain1 = 1
        iMain = iMain + 1
        return TimeSetMain1
    
    if(Train[1]=="32" and StartMain2==0):
        now = datetime.datetime.now()
        TimeSetMain2 = now + datetime.timedelta(seconds=TT2[iMain])
        StartMain2 = 1
        iMain = iMain + 1
        return TimeSetMain2
    
    if(Train[2]=="32" and StartMain3==0):
        now = datetime.datetime.now()
        TimeSetMain3 = now + datetime.timedelta(seconds=TT2[iMain])
        StartMain3 = 1
        iMain = iMain + 1
        return TimeSetMain3
    
    if(Train[3]=="32" and StartMain4==0):
        now = datetime.datetime.now()
        TimeSetMain4 = now + datetime.timedelta(seconds=TT2[iMain])
        StartMain4 = 1
        iMain = iMain + 1
        return TimeSetMain4

def DispatchSubReset():
    global StartSub1
    global StartSub2
    global StartSub3
    global StartSub4
    global StartMain1
    global StartMain2
    global StartMain3
    global StartMain4
    global FG22_32
    
    if(Train[0]=="12"):
        StartSub1 = 0
        StartMain1 = 0
        FG22_32 = 1     # <K4>
        return
    
    if(Train[1]=="2"):
        StartSub2 = 0
        StartMain2 = 0
    
    if(Train[2]=="2"):
        StartSub3 = 0
        StartMain3 = 0
    
    if(Train[3]=="2"):
        StartSub4 = 0
        StartMain4 = 0

def startWaite(t_now, SetSubtime):
    global StartSub1
    
    if(StartSub1 == 1):
        if(SetSubtime < t_now):
            ContFSlow1()
            return
    
def Decelerate():
    global FG13_21
    
    if((Train[0]=="13" or Train[1]=="13" or Train[2]=="13" or Train[3]=="13" or
       Train[0]=="21" or Train[1]=="21" or Train[2]=="21" or Train[3]=="21") and
       (FG13_21==0)):
        ContFMSlow1()
        FG13_21 = 1

def stopCont():
    global FG22_32
    global FG13_21

    if(Train[0]=="22" and FG22_32==1):
        ContStop1()
        StartSub1 = 0
        FG22_32 = 0      # <K5>
        FG13_21 = 0      # <K6>
    if(Train[1]=="22" and FG22_32==1):
        ContStop2()
        FG22_32 = 0      # <K5>
        FG13_21 = 0      # <K6>
    if(Train[2]=="22" and FG22_32==1):
        ContStop3()
        FG22_32 = 0      # <K5>
        FG13_21 = 0      # <K6>
    if(Train[3]=="22" and FG22_32==1):
        ContStop4()
        FG22_32 = 0      # <K5>
        FG13_21 = 0      # <K6>
    if(Train[0]=="32" and FG22_32==1):
        ContStop1()
        FG22_32 = 0      # <K5>
        FG13_21 = 0      # <K6>
    if(Train[1]=="32" and FG22_32==1):
        ContStop2()
        FG22_32 = 0      # <K5>
        FG13_21 = 0      # <K6>
    if(Train[2]=="32" and FG22_32==1):
        ContStop3()
        FG22_32 = 0      # <K5>
        FG13_21 = 0      # <K6>
    if(Train[3]=="32" and FG22_32==1):
        ContStop4()
        FG22_32 = 0      # <K5>
        FG13_21 = 0      # <K6>

def OutPointJudg():
    global TimeSetSub1
    global TimeSetMain1
    now = datetime.datetime.now()
    if not(TimeSetSub1==None or TimeSetMain1==None):
        if(TimeSetSub1 < TimeSetMain1):
            serMEGA.write(b"S\n")
    
    if not(TimeSetSub1==None or TimeSetMain1==None):
        if(TimeSetSub1 > TimeSetMain1):
            serMEGA.write(b"M\n")
    

def sendTWELite(stick, sendto = 0x00,
        digital = [-1, -1, -1, -1],
        analog  = [-1, -1, -1, -1]):
    # 先頭3バイト
    data = [sendto, 0x80, 0x01]
    
    # デジタル出力
    do = 0
    domask = 0
    for index, value in enumerate(digital):
        if value >= 0:
            domask |= 1 << index
            do |= (value & 1) << index
    data.append(do)
    data.append(domask)
    
    # アナログ出力
    for index, value in enumerate(analog):
        if value >= 0 and value <= 100:
            v = int(1024 * value / 100)
            data.append(v >> 8)
            data.append(v & 0xff)
        else:
            data.append(0xff)
            data.append(0xff)

    # チェックサムを計算する
    chksum = 0
    for val in data:
        chksum = (chksum + val) & 0xff
    data.append((0x100 - chksum) & 0xff)

    # 16進数文字列に変換する
    ss = struct.Struct("14B")
    outstring = str(binascii.hexlify(ss.pack(*data)), 'utf-8').upper()

    # TWE-Liteに送信する
    stick.write(bytes(":" + outstring + "\r\n", 'utf-8'))
    return

# コマンド0x81を解釈する関数
def parseTWELite(data):
    # バイトデータに変換する
    ss = struct.Struct(">BBBBBIBHBHBBBBBBBBB")
    data = binascii.unhexlify(data.rstrip())
    parsed = ss.unpack(data)

    # デジタル入力／アナログ入力の値を計算する
    digital = [0] * 4
    digitalchanged = [0] * 4
    analog = [0xffff] * 4

    for i in range(4):
        # デジタル入力
        if parsed[11] & (1 << i):
            digital[i] = 1
        else:
            digital[i] = 0
        if parsed[12] & (1 << i):
            digitalchanged[i] = 1
        else:
            digitalchanged[i] = 0

        # アナログ入力
        if parsed[13 + i] == 0xff :
            analog[i] = 0xffff
        else:
            analog[i] = (parsed[13 + i] * 4 + ((parsed[17] >> (2 << i)) & 3)) * 4

    # 結果を返す
    result = {
        "from" : parsed[0],
        "lqi" : parsed[4],
        "fromid" : parsed[5],
        "to": parsed[6],
        "timestamp": parsed[7],
        "isrelay": parsed[8],
        "baterry" : parsed[9],
        "digital" : digital,
        "digitalchanged" : digitalchanged,
        "analog" : analog
    }
    return result


def ContFFast1():
    # 前進高速
    sendTWELite(stick, sendto = 0x01, digital=[0, 1, 0, 1], analog=[0, 80, -1, -1])
    return

def ContFSlow1():
    # 前進低速
    sendTWELite(stick, sendto = 0x01, digital=[0, 1, 0, 1], analog=[0, 50, -1, -1])
    return

def ContFMSlow1():
    # 前進超低速
    sendTWELite(stick, sendto = 0x01, digital=[0, 1, 0, 1], analog=[0, 39, -1, -1])
    return

def ContRSlow1():
    # 後進低速
    sendTWELite(stick, sendto = 0x01, digital=[1, 0, 1, 0], analog=[60, 0, -1, -1])
    return

def ContRFast1():
    # 後進高速
    sendTWELite(stick, sendto = 0x01, digital=[1, 0, 1, 0], analog=[80, 0, -1, -1])
    return

def ContStop1():
    # 停止
    sendTWELite(stick, sendto = 0x01, analog=[100, 100, -1, -1], digital=[1, 1, 1, 1])
    return

def readline1():
    # 受信（TWELITEからデータ読み込み）
    data = stick.readline()
    # 先頭の":"を取り除く
    data = data[1:]
    # TWELITEからの正常送信文字数をチェック（ゴミデータ排除）
    if(len(data)==50):
        parsed = parseTWELite(data)
        baterry = (parsed["analog"][0])
        distance = (parsed["analog"][1])
        print(baterry)
        print(distance)
# * Train[1]-END *******************************************************

# * 列車制御　Train[2] **************************************************************
def ContFFast2():
    # 前進高速
    sendTWELite(stick, sendto = 0x02, digital=[0, 1, 0, 1], analog=[0, 80, -1, -1])
    return

def ContFSlow2():
    # 前進低速
    sendTWELite(stick, sendto = 0x02, digital=[0, 1, 0, 1], analog=[0, 55, -1, -1])
    return

def ContFMSlow2():
    # 前進超低速
    sendTWELite(stick, sendto = 0x02, digital=[0, 1, 0, 1], analog=[0, 43, -1, -1])
    return

def ContRSlow2():
    # 後進低速
    sendTWELite(stick, sendto = 0x02, digital=[1, 0, 1, 0], analog=[60, 0, -1, -1])
    return

def ContRFast2():
    # 後進高速
    sendTWELite(stick, sendto = 0x02, digital=[1, 0, 1, 0], analog=[80, 0, -1, -1])
    return

def ContStop2():
    # 停止
    sendTWELite(stick, sendto = 0x02, analog=[100, 100, -1, -1], digital=[1, 1, 1, 1])
    return

def readline2():
    # 受信（TWELITEからデータ読み込み）
    data = stick.readline()
    # 先頭の":"を取り除く
    data = data[1:]
    # TWELITEからの正常送信文字数をチェック（ゴミデータ排除）
    if(len(data)==50):
        parsed = parseTWELite(data)
        baterry = (parsed["analog"][0])
        distance = (parsed["analog"][1])
        print(baterry)
        print(distance)
# * Train[2]-END *******************************************************

# * 列車制御　Train[3] **************************************************************
def ContFFast3():
    # 前進高速
    sendTWELite(stick, sendto = 0x03, digital=[0, 1, 0, 1], analog=[0, 80, -1, -1])
    return

def ContFSlow3():
    # 前進低速
    sendTWELite(stick, sendto = 0x03, digital=[0, 1, 0, 1], analog=[0, 55, -1, -1])
    return

def ContFMSlow3():
    # 前進超低速
    sendTWELite(stick, sendto = 0x03, digital=[0, 1, 0, 1], analog=[0, 40, -1, -1])
    return

def ContRSlow3():
    # 後進低速
    sendTWELite(stick, sendto = 0x03, digital=[1, 0, 1, 0], analog=[60, 0, -1, -1])
    return

def ContRFast3():
    # 後進高速
    sendTWELite(stick, sendto = 0x03, digital=[1, 0, 1, 0], analog=[80, 0, -1, -1])
    return

def ContStop3():
    # 停止
    sendTWELite(stick, sendto = 0x03, analog=[100, 100, -1, -1], digital=[1, 1, 1, 1])
    return

def readline3():
    # 受信（TWELITEからデータ読み込み）
    data = stick.readline()
    # 先頭の":"を取り除く
    data = data[1:]
    # TWELITEからの正常送信文字数をチェック（ゴミデータ排除）
    if(len(data)==50):
        parsed = parseTWELite(data)
        baterry = (parsed["analog"][0])
        distance = (parsed["analog"][1])
        print(baterry)
        print(distance)
# * Train[3]-END *******************************************************

# * 列車制御　Train[4] **************************************************************
def ContFFast4():
    # 前進高速
    sendTWELite(stick, sendto = 0x04, digital=[0, 1, 0, 1], analog=[0, 80, -1, -1])
    return

def ContFSlow4():
    # 前進低速
    sendTWELite(stick, sendto = 0x04, digital=[0, 1, 0, 1], analog=[0, 55, -1, -1])
    return

def ContFMSlow4():
    # 前進超低速
    sendTWELite(stick, sendto = 0x04, digital=[0, 1, 0, 1], analog=[0, 40, -1, -1])
    return

def ContRSlow4():
    # 後進低速
    sendTWELite(stick, sendto = 0x04, digital=[1, 0, 1, 0], analog=[60, 0, -1, -1])
    return

def ContRFast4():
    # 後進高速
    sendTWELite(stick, sendto = 0x04, digital=[1, 0, 1, 0], analog=[80, 0, -1, -1])
    return

def ContStop4():
    # 停止
    sendTWELite(stick, sendto = 0x04, analog=[100, 100, -1, -1], digital=[1, 1, 1, 1])
    return

def readline4():
    data = stick.readline()
    data = data[1:]

    if(len(data)==50):
        parsed = parseTWELite(data)
        baterry = (parsed["analog"][0])
        distance = (parsed["analog"][1])
        print(baterry)
        print(distance)


def trainPoji():
    serMEGA.write(b"R\n")
    for i in range(0, 5):
        Train[i] = serMEGA.readline().decode('utf-8').rstrip()

while 1:
    trainPoji()
    if(StartSub1 == 0):
        SetSubtime = DispatchSub()
    t_now = datetime.datetime.now()
    OutPointJudg()
    startWaite(t_now, SetSubtime)
#    readline()
    DispatchSubReset()
    Decelerate()
    stopCont()
    if(iSub > 7):
        iSub = 0
    if(iMain > 7):
        iMain = 0

stick.close()
serMEGA.close()
GPIO.cleanup()
