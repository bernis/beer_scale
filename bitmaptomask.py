import PIL.Image
import numpy as np
import glob

T = np.array([255, 0, 255, 255])
files = glob.glob("*.png")

with open("sketch/beer_gfx.h", 'w') as out:
    for f in files:
        print(f)
        img = PIL.Image.open(f)
        pix = np.array(img)
        
        # border
        brd = (pix == T).all(axis=2).sum(axis=0)
        
        # rgb565
        pix565 = np.zeros(pix.shape[:-1], dtype=np.uint16)
        pix565 += (np.minimum(pix[:,:,0].astype(np.uint16) + 4, 255) / 8).astype(np.uint16) * 8 * 256 
        pix565 += (np.minimum(pix[:,:,1].astype(np.uint16)  + 2, 255) / 4).astype(np.uint16) * 4 * 8
        pix565 += (np.minimum(pix[:,:,2].astype(np.uint16)  + 4, 255) / 8).astype(np.uint16) 

        # print
        name = f.split(".")[0].upper()
        length = len(brd)
        data = ",".join(map(str,brd))
        out.write(f"static const PROGMEM uint8_t BEER_{name}_BRD[{length}] = {{{data}}};\n")
        out.write(f"static const PROGMEM uint16_t BEER_{name}_IMG[] = {{\n")
        for row in range(pix565.shape[0]):
            line = ", ".join([f"0x{p:04x}" for p in pix565[row,:]])
            line += "," if row < (pix565.shape[0]-1) else ''
            out.write(f"  {line}\n")
        out.write("};\n")
        #img.show()
