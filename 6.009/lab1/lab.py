#!/usr/bin/env python3

import sys
import math
import base64
import tkinter

from io import BytesIO
from PIL import Image as PILImage

## NO ADDITIONAL IMPORTS ALLOWED!

class Image:
    def __init__(self, width, height, pixels):
        self.width = width
        self.height = height
        self.pixels = pixels

    def get_pixel(self, x, y):
        #if pixel is outside the boundaries of the image, return the "extended" value
        if x<0:
            x=0
        elif x>=self.width:
            x=self.width-1
        if y<0:
            y=0
        elif y>=self.height:
            y=self.height-1
        #given pixel locations within image, find and return that pixel
        return self.pixels[(x+y*self.width)]

    def set_pixel(self, x, y, c):
        self.pixels[(x+y*self.width)] = c

    def apply_per_pixel(self, func):
        #create a copy of the image
        result = Image.new(self.width, self.height)
        #for every pixel in the image...
        for x in range(result.width):
            for y in range(result.height):
                #retrieve the pixel's current value
                color = self.get_pixel(x, y)
                #calculate the new value with the given function
                newcolor = func(color)
                #assign the new value to that pixel
                result.set_pixel(x,y, newcolor)
        return result

    def inverted(self):
        #change the value of every pixel to its "opposite" within the 0 to 255 range
        result = self.apply_per_pixel(lambda c: 255-c)
        #clip the values to make sure they are within the 0 to 255 range
        return result.clip()

    def blurred(self, n):
        #copy the image
        result = Image(self.width, self.height,self.pixels.copy())
        # create an nxn kernel in which every value is 1/n^2
        kernel = [(1/n**2) for i in range(n*n)]
        #apply the kernel to the image, and clip and round the pixels
        result = result.apply_kernel(kernel)
        result=result.clip()
        result = result.round_pix()
        return result

    
    #to "unsharpen" an image, we want essentially the negative of the blur kernel, with 2 added
    #to the central element (aka multiply pixel by 2, subtract the blurred value)
    def sharpened(self, n):
        #copy image
        result = Image(self.width, self.height,self.pixels.copy())
        #create the same kernel as in the blur method, only with negative values since we want to subtract it
        kernel = [-(1/n**2) for i in range(n*n)]
        #locate the center of the kernel and add 2 to its value
        kernel[int((n**2-1)/2)] +=2
        #apply the unsharpen kernel to the image and clip and round the pixels
        result = result.apply_kernel(kernel)
        result = result.clip()
        result = result.round_pix()
        return result

    def edges(self):
        #create the given kx and ky kernels
        kx = [-1, 0, 1, -2, 0, 2, -1, 0, 1]
        ky = [-1, -2, -1, 0, 0, 0,1, 2, 1]
        #create 3 copies of the image, two to apply the given kernels separately and one for the final result
        ox = Image(self.width, self.height,self.pixels.copy())
        oy = Image(self.width, self.height,self.pixels.copy())
        result = Image.new(self.width, self.height)
        #apply the kernels separately
        ox = ox.apply_kernel(kx)
        oy = oy.apply_kernel(ky)
        #use the given formula to find the desired image from the two transformed images, then clip and round
        result.pixels = [((x**2+y**2)**0.5) for x,y in zip(ox.pixels, oy.pixels)]
        result = result.clip()
        result = result.round_pix()
        return result
        
        
        
        
        
    def clip(self):
        #trim the pixel values if they go over or under the 0 to 255 range
        return self.apply_per_pixel(lambda c: 0 if c<0 else 255 if c>255 else c)
    
    def round_pix(self):
        #round every pixel in the image
        return self.apply_per_pixel(lambda c: round(c))
    
    def apply_kernel(self, kernel):
        #store a copy of the image
        result = Image(self.width, self.height,self.pixels.copy())
        #calculate the side length of the square kernel
        k_size = int(len(kernel)**0.5)     
        layers = int((k_size-1)/2)
        for y in range(result.height):
            for x in range(result.width):
                new_val = 0
                count = 0
                #indexing each kernel value relative to the center
                for ky in range(-layers, layers+1):
                    for kx in range(-layers, layers+1):
                        new_val+=self.get_pixel(x+kx, y+ky)*kernel[count]
                        count+= 1
                result.set_pixel(x,y,new_val)
        #result = result.clip()
        return result
                        
        


    # Below this point are utilities for loading, saving, and displaying
    # images, as well as for testing.

    def __eq__(self, other):
        return all(getattr(self, i) == getattr(other, i)
                   for i in ('height', 'width', 'pixels'))

    def __repr__(self):
        return "Image(%s, %s, %s)" % (self.width, self.height, self.pixels)

    @classmethod
    def load(cls, fname):
        """
        Loads an image from the given file and returns an instance of this
        class representing that image.  This also performs conversion to
        grayscale.

        Invoked as, for example:
           i = Image.load('test_images/cat.png')
        """
        with open(fname, 'rb') as img_handle:
            img = PILImage.open(img_handle)
            img_data = img.getdata()
            if img.mode.startswith('RGB'):
                pixels = [round(.299*p[0] + .587*p[1] + .114*p[2]) for p in img_data]
            elif img.mode == 'LA':
                pixels = [p[0] for p in img_data]
            elif img.mode == 'L':
                pixels = list(img_data)
            else:
                raise ValueError('Unsupported image mode: %r' % img.mode)
            w, h = img.size
            return cls(w, h, pixels)

    @classmethod
    def new(cls, width, height):
        """
        Creates a new blank image (all 0's) of the given height and width.

        Invoked as, for example:
            i = Image.new(640, 480)
        """
        return cls(width, height, [0 for i in range(width*height)])

    def save(self, fname, mode='PNG'):
        """
        Saves the given image to disk or to a file-like object.  If fname is
        given as a string, the file type will be inferred from the given name.
        If fname is given as a file-like object, the file type will be
        determined by the 'mode' parameter.
        """
        out = PILImage.new(mode='L', size=(self.width, self.height))
        out.putdata(self.pixels)
        if isinstance(fname, str):
            out.save(fname)
        else:
            out.save(fname, mode)
        out.close()

    def gif_data(self):
        """
        Returns a base 64 encoded string containing the given image as a GIF
        image.

        Utility function to make show_image a little cleaner.
        """
        buff = BytesIO()
        self.save(buff, mode='GIF')
        return base64.b64encode(buff.getvalue())

    def show(self):
        """
        Shows the given image in a new Tk window.
        """
        global WINDOWS_OPENED
        if tk_root is None:
            # if tk hasn't been properly initialized, don't try to do anything.
            return
        WINDOWS_OPENED = True
        toplevel = tkinter.Toplevel()
        # highlightthickness=0 is a hack to prevent the window's own resizing
        # from triggering another resize event (infinite resize loop).  see
        # https://stackoverflow.com/questions/22838255/tkinter-canvas-resizing-automatically
        canvas = tkinter.Canvas(toplevel, height=self.height,
                                width=self.width, highlightthickness=0)
        canvas.pack()
        canvas.img = tkinter.PhotoImage(data=self.gif_data())
        canvas.create_image(0, 0, image=canvas.img, anchor=tkinter.NW)
        def on_resize(event):
            # handle resizing the image when the window is resized
            # the procedure is:
            #  * convert to a PIL image
            #  * resize that image
            #  * grab the base64-encoded GIF data from the resized image
            #  * put that in a tkinter label
            #  * show that image on the canvas
            new_img = PILImage.new(mode='L', size=(self.width, self.height))
            new_img.putdata(self.pixels)
            new_img = new_img.resize((event.width, event.height), PILImage.NEAREST)
            buff = BytesIO()
            new_img.save(buff, 'GIF')
            canvas.img = tkinter.PhotoImage(data=base64.b64encode(buff.getvalue()))
            canvas.configure(height=event.height, width=event.width)
            canvas.create_image(0, 0, image=canvas.img, anchor=tkinter.NW)
        # finally, bind that function so that it is called when the window is
        # resized.
        canvas.bind('<Configure>', on_resize)
        toplevel.bind('<Configure>', lambda e: canvas.configure(height=e.height, width=e.width))

        # when the window is closed, the program should stop
        toplevel.protocol('WM_DELETE_WINDOW', tk_root.destroy)


try:
    tk_root = tkinter.Tk()
    tk_root.withdraw()
    tcl = tkinter.Tcl()
    def reafter():
        tcl.after(500,reafter)
    tcl.after(500,reafter)
except:
    tk_root = None
WINDOWS_OPENED = False

if __name__ == '__main__':
    # code in this block will only be run when you explicitly run your script,
    # and not when the tests are being run.  this is a good place for
    # generating images, etc.
#    i = Image.load('test_images/cat.png')
#    i.show()
    pass

    # the following code will cause windows from Image.show to be displayed
    # properly, whether we're running interactively or not:
    if WINDOWS_OPENED and not sys.flags.interactive:
        tk_root.mainloop()
