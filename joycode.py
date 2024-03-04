# bare-bones wire protocol for stuffing control data to hub

"""
from uselect import poll
from usys import stdin

poller = poll()
# Register the standard input so we can read keyboard presses.
poller.register(stdin)

# create protocol with three variables using 2-bytes on the wire
j = JoyProto(['coll', 'pitch', 'roll'], 2, poller, stdin)
"""
class JoyProtocol(object):
    def __init__(self, varnames, wirewidth, poller, stream, varwidth=8):
        self.varnames = varnames
        self.wirewidth = wirewidth # number of hex digits
        self.fmt = f"%%%0{wirewidth}xx" % wirewidth
        self.varwidth = varwidth # number of bits coming from game controller variable dict
        self.varscale = 2**varwidth/2.0 # 11: 1024.0, 8: 128.0
        self.varthresh = 5/self.varscale # ignore smaller inputs
        
        self.valdict = {} # results dictionary
        for n in self.varnames: 
            self.valdict[n] = 0
        self.prototype = self.encode(self.valdict) 
        self.protolen = len(self.prototype)
        
        # io stuff
        self.keyboard = poller
        self.stream = stream
        self.buf = list("x" * self.protolen) # control input buffer
        self.pos = 0              # control input buffer pointer
        
        #cache variable offsets
        self.nvars = int((self.protolen-2)/self.varwidth) # should equal len(self.varnames)
        self.varoffs = [self.wirewidth*i+1 for i in range(self.nvars)]
        
    def encode(self, valdict):
        "encode valdict for the wire, with self.width-wide hex digits"
        return ">"+''.join([self.fmt % valdict[n] for n in self.varnames])+"<" 
        
    def hbytes(self, arr, off):
        "decode self.width big-endian hex digits from array[offset] to unsigned int"
        n = self.wirewidth
        return sum([(16**(n-i-1)*int(arr[off+i])) for i in range(n)]) 
        
    def decode_js(self, js_uint):
        "scale self.varwidth-bit unsigned int into range [-1.0,1.0], with deadzone"   
        jsf = js_uint/self.varscale - 1
        if(abs(jsf)<self.varthresh): # control the dead zone
            jsf = 0
        return jsf 
        
    def decode_wire(self):
        for i in range(self.nvars):
            n = self.varnames[i]
            offset = self.varoffs[i]
            self.valdict[n] = self.decode_js(self.hbytes(self.buf, offset))
            
    def poll(self):
      # actually reads the input buffer
      if self.keyboard.poll(0):
          # Read the key and print it.
          key = self.stream.read(1)
          if key == '>':
              self.pos = 0
          self.buf[self.pos] = key
          self.pos += 1
          if (self.pos==self.protolen):
              self.pos = 0
              if (key == '<'):
                  return True 
              else:
                  print("buffer bad")
      return False  

def test():
    """
    >>> test()
    Prototype: >0000000000<, length: 12
    varnames: ['coll', 'roll', 'pitch', 'yaw', 'glyph'] wirewidth: 2
    indict: {'coll': 8, 'roll': 16, 'pitch': 32, 'yaw': 64, 'glyph': 128}
    encin: >0810204080<
    outdict: {'coll': -0.9375, 'roll': 0, 'pitch': 0, 'yaw': 0, 'glyph': 0}
    >>>
    """
    
    import sys
    j = JoyProtocol(['coll', 'roll', 'pitch', 'yaw', 'glyph'], 2, None, sys.stdin)
    print(f"Prototype: {j.prototype}, length: {j.protolen}") 
    print(f"varnames: {j.varnames} wirewidth: {j.wirewidth}") 
    indict = {'coll':8, 'roll':16, 'pitch':32, 'yaw':64, 'glyph':128}
    encin = j.encode(indict)
    j.buf = encin
    j.decode_wire()
    outdict = j.valdict
    print(f"indict: {indict}\nencin: {encin}\noutdict: {outdict}")

if __name__ == "__main__":
    test()
