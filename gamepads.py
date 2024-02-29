def dword(arr, off):
    i = arr[off] + 256*arr[off+1]
    return i 

def decode_js(js_uint, basis=1024.0): # 1024.0 or 128.0
    # scale 8-bit unsigned into range [-1.0,1.0]
    # both axes reversed logically, so going up and right decrease
    jsf = js_uint/basis - 1
    jsf = -jsf
    return jsf   

# Taranis major axes 3 5 7 9 lx = 9,A, ly = 7,8, rx = 3,4, ry = 5,6 LSB,MSB 0 .. 2047 
#          rx,3,4,AEL,ch1 ry,5,6,ELE,ch2 ly,7,8,THR,ch3 lx,9,A,RUD,ch4
     
def decode_taranis(report):
    spin = dword(report,9) # spin / lx / yaw
    crab = dword(report,3) # crab / rx / roll
    fwdback = dword(report,5) # fb / ry / pitch
    coll = dword(report,7) # collective, ly
    
    yf = spin #/2048.0
    rf = crab #/2048.0
    pf = fwdback #/2048.0
    cf = coll #/2048.0
    #print(f"cf: {cf}  rf: {rf}  pf: {pf}  yf: {yf}  ")
    SA = dword(report, 11)
    SB = dword(report, 13)
    SC = dword(report, 15)
    SD = dword(report, 17)
    
    #print(f"SA:{SA}, SB:{SB}, SC:{SC}, SD{SD}")
    if(SA!=2047):
        glyph = 24 # square
    elif(SB!=2047):
        glyph = 40 # X
    elif(SC!=2047):
        glyph = 72 # O
    elif(SD!=2047):
        glyph = 68 # Triangle, I think
    else:
        glyph = 0
    
    crab = int(crab/8)  
    spin = int(spin/8)    
    fwdback = 255-int(fwdback/8) # match ps4 order  
      
    #crab = decode_js(crab, 1024.0)
    #spin = decode_js(spin, 1024.0)  
    #fwdback = decode_js(fwdback, 1024.0)

    return {'crab': crab, 'spin': spin, 'fwdback': fwdback, 'glyph': glyph,
            'yaw': yf, 'pitch': pf, 'roll': rf, 'coll': cf}

# PS4 major axes lx = 1, ly = 2, rx = 3, ry = 4   
 
def decode_ps4(report):
    crab = report[1] # crab / lx
    spin = report[3] # spin / rx
    fwdback = report[4] # fb / ry
    glyph = report[5]

    #crab = decode_js(crab, 128.0)
    #spin = decode_js(spin, 128.0)  
    #fwdback = decode_js(fwdback, 128.0)

    return {'crab': crab, 'spin': spin, 'fwdback': fwdback, 'glyph': glyph}