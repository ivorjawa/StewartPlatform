# game controller interface using USB HID.
# this started with a PS4 controller, but it kinda sucked
# I'm using a FrSky Taranis x9d connected via USB now.
# Controller frustration is annoying
# if you want to use a PS4 controller, the code should be 
# easy to adapt back

import hid

def open_taranis(debug=False):
    #https://blog.thea.codes/talking-to-gamepads-without-pygame/
    # decoding ps4
    # https://web.archive.org/web/20210301230721/https://www.psdevwiki.com/ps4/DS4-USB

    if(debug):
        for device in hid.enumerate():
            print(f"0x{device['vendor_id']:04x}:0x{device['product_id']:04x}   {device['product_string']}")

    ctrlr = hid.device()

    #gamepad.open(0x054c, 0x09cc) # crappy PS4 controller
    #from gamepads import decode_ps4 as decode_report

    print("Opening Taranis")
    ctrlr.open(0x1209, 0x4f54) # Taranis

    ctrlr.set_nonblocking(True)
    return ctrlr
    
def dword(arr, off):
    # decode little-endian word
    return arr[off] + 256*arr[off+1]

# Taranis major axes 3 5 7 9 lx = 9,A, ly = 7,8, rx = 3,4, ry = 5,6 LSB,MSB 0 .. 2047 
#          rx,3,4,AEL,ch1 ry,5,6,ELE,ch2 ly,7,8,THR,ch3 lx,9,A,RUD,ch4

# for aircraft: yaw, roll, pitch, collective
# for mechanum: spin, crab, forward-back, unused
def decode_taranis(report):
    yaw = dword(report,9) # spin / lx / yaw
    roll = 2047-dword(report,3) # crab / rx / roll
    pitch = dword(report,5) # fwdback / ry / pitch
    coll = dword(report,7) # collective, ly
    
    #print(f"cf: {cf}  rf: {rf}  pf: {pf}  yf: {yf}  ")
    SA = dword(report, 11)
    SB = dword(report, 13)
    SC = dword(report, 15)
    SD = dword(report, 17)
    
    
    #print(f"SA:{SA}, SB:{SB}, SC:{SC}, SD{SD}")
    glyph = 0 # 24: Square, 40: X, 72: O, 68: Triangle
    if(SA==1024):
        glyph |= 24 # square
    if(SB==1024):
        glyph |= 40 # X
    if(SC==1024):
        glyph |= 72 # O
    if(SD==1024):
        glyph |= 68 # Triangle

    
    # scaled from 11 to 8 bits
    scale = 8
    roll = int(roll/scale) 
    yaw = int(yaw/scale)    
    pitch = int(pitch/scale) 
    coll = int(coll/scale) 

    return {'yaw': yaw, 'pitch': pitch, 'roll': roll, 'coll': coll, 'glyph': glyph}

# PS4 major axes lx = 1, ly = 2, rx = 3, ry = 4   

# for aircraft: yaw, roll, pitch, collective
# for mechanum: spin, crab, forward-back, unused 
def decode_ps4(report):
    #yaw = spin/lx, coll = coll/ly, roll = crab/rx, pitch = fwdback/ry
    # unsigned 1-byte
    roll = report[1] # roll / lx
    coll = report[2] # coll/ ry
    yaw = report[3] # yaw / rx
    pitch = report[4] # pitch / ry
    glyph = report[5]

    return {'yaw': yaw, 'pitch': pitch, 'roll': roll, 'coll': coll, 'glyph': glyph}

