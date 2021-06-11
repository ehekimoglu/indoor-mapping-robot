#save file as .eps
#need to import "pillow" package
from PIL import Image
import turtle


t = turtle.Turtle()
t.forward(100)
ts = turtle.getscreen()
ts.getcanvas().postscript(file="deneme.eps")

fileName = 'deneme.eps'
screen = turtle.Screen()
screen.setup(1000, 1000)
turtle = turtle.Turtle(visible=False)
screen.tracer(False)
screen.tracer(True)
canvas = screen.getcanvas()
canvas.postscript(file=fileName)
canvas.postscript(file=fileName, width=1000, height=1000)

#converting .eps to .jpg
img = Image.open(fileName)
img.save("deneme.jpg", "JPEG")