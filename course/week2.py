# 開始透過 ggame 程式庫進行網際繪圖
# ggame 手冊
# http://brythonserver.github.io/ggame/
'''
from ggame import App, Color, LineStyle, Sprite
from ggame import RectangleAsset, CircleAsset, EllipseAsset, PolygonAsset

# Three primary colors with no transparency (alpha = 1.0)
red = Color(0xff0000, 1.0)
green = Color(0x00ff00, 1.0)
blue = Color(0x0000ff, 1.0)
black = Color(0x000000, 1.0)

# Define a line style that is a thin (1 pixel) wide black line
thinline = LineStyle(1, black)
# A graphics asset that represents a rectangle
rectangle = RectangleAsset(50, 20, thinline, blue)

# Now display a rectangle
Sprite(rectangle)

myapp = App()
myapp.run()
'''
'''
###############################################
from ggame import App, ImageAsset, Sprite

# Create a displayed object at 0,0 using an image asset
Sprite(ImageAsset("ggame/bunny.jpg"), (0,0))
# Create the app, with a 500x500 pixel stage
app = App(500,500)  
# Run the app
app.run()
'''
'''
###############################################
from ggame import App, Color, LineStyle, Sprite
from ggame import RectangleAsset, CircleAsset, EllipseAsset, PolygonAsset

# Three primary colors with no transparency (alpha = 1.0)
white = Color(0xE3E3E3, 1.0)
yellow = Color(0xEDF55F, 1.0)
blue = Color(0x0000FF, 1.0)
green = Color(0x84F564, 1.0)
black = Color(0x4D4843, 1.0)

# Define a line style that is a thin (1 pixel) wide black line
thinline = LineStyle(1, black)
# A graphics asset that represents a rectangle
rectangle = RectangleAsset(400, 200, thinline, green)
ellipse = EllipseAsset(30, 30, thinline, yellow)
ellipse2 = EllipseAsset(70, 40, thinline, blue)
polygon = PolygonAsset([(-85, 100), (25, -75), (100, 125), (-85, 100)], thinline, black)
polygon2 = PolygonAsset([(100, 100), (225, -125), (350, 125), (100, 100)], thinline, black)
# Now display a rectangle
Sprite(rectangle, (100, 200))
Sprite(ellipse, (75, 40))
Sprite(ellipse2, (325, 325))
Sprite(polygon, (150, 150))
Sprite(polygon2, (150, 150))

myapp = App()
myapp.run()
'''
#############################################
"""
picture.py
Author: <Hayden Hatfield>
Credit: <list sources used, if any>

Assignment:Picture

Use the ggame library to "paint" a graphical picture of something (e.g. a house, a face or landscape).

Use at least:
1. Three different Color objects.
2. Ten different Sprite objects.
3. One (or more) RectangleAsset objects.
4. One (or more) CircleAsset objects.
5. One (or more) EllipseAsset objects.
6. One (or more) PolygonAsset objects.

See:
https://github.com/HHS-IntroProgramming/Standards-and-Syllabus/wiki/Displaying-Graphics
for general information on how to use ggame.

See:
http://brythonserver.github.io/ggame/
for detailed information on ggame.

"""
'''
from ggame import App, Color, LineStyle, Sprite, RectangleAsset, CircleAsset, EllipseAsset, PolygonAsset

# add your code here \/  \/  \/

red = Color(0xff0000, 1.0)
green = Color(0x00ff00, 1.0)
blue = Color(0x0000ff, 1.0)
black = Color(0x000000, 1.0)
darkgreen = Color(0x0e270f, 1.0)
nightsky = Color(0x16152b, 1.0)
ground = Color(0x06471a, 1.0)
wood = Color(0x542a0c, 1.0)
moonwhite = Color(0xfff3aa, 1.0)
grey = Color(0x818181, 1.0)
line = LineStyle(2, black)


trunk = RectangleAsset( 20, 50, line, wood )
tri13 = PolygonAsset([(0,100),(200,100),(100,0)], line, darkgreen )
tri14 = PolygonAsset([(0,100),(200,100),(100,0)], line, darkgreen )
tri15 = PolygonAsset([(0,100),(200,100),(100,0)], line, darkgreen )
tri16 = PolygonAsset([(0,100),(200,100),(100,0)], line, darkgreen )
trunk = RectangleAsset( 20, 50, line, wood )
tri9 = PolygonAsset([(0,100),(200,100),(100,0)], line, darkgreen )
tri10 = PolygonAsset([(0,100),(200,100),(100,0)], line, darkgreen )
tri11 = PolygonAsset([(0,100),(200,100),(100,0)], line, darkgreen )
tri12 = PolygonAsset([(0,100),(200,100),(100,0)], line, darkgreen )
trunk = RectangleAsset( 20, 50, line, wood )
tri5 = PolygonAsset([(0,100),(200,100),(100,0)], line, darkgreen )
tri6 = PolygonAsset([(0,100),(200,100),(100,0)], line, darkgreen )
tri7 = PolygonAsset([(0,100),(200,100),(100,0)], line, darkgreen )
tri8 = PolygonAsset([(0,100),(200,100),(100,0)], line, darkgreen )
trunk = RectangleAsset( 20, 50, line, wood )
tri1 = PolygonAsset([(0,100),(200,100),(100,0)], line, darkgreen )
tri2 = PolygonAsset([(0,100),(200,100),(100,0)], line, darkgreen )
tri3 = PolygonAsset([(0,100),(200,100),(100,0)], line, darkgreen )
tri4 = PolygonAsset([(0,100),(200,100),(100,0)], line, darkgreen )
moon = EllipseAsset(40, 40, line, moonwhite )
rock = CircleAsset(75, line, grey)
background = RectangleAsset( 1000, 1000, line, nightsky )
ground = RectangleAsset( 1000, 100, line, ground )

Sprite(background, (0, 0))
Sprite(tri9, (450, 575))
Sprite(tri10, (450, 525))
Sprite(tri11, (450, 475))
Sprite(tri12, (450, 425))
Sprite(trunk, (540, 675))

Sprite(tri13, (250, 550))
Sprite(tri14, (250, 500))
Sprite(tri15, (250, 450))
Sprite(tri16, (250, 400))
Sprite(trunk, (340, 650))

Sprite(tri5, (150, 575))
Sprite(tri6, (150, 525))
Sprite(tri7, (150, 475))
Sprite(tri8, (150, 425))
Sprite(trunk, (240, 675))

Sprite(tri1, (20, 550))
Sprite(tri2, (20, 500))
Sprite(tri3, (20, 450))
Sprite(tri4, (20, 400))
Sprite(trunk, (110, 650))
Sprite(moon, (900, 50))
Sprite(rock, (925, 700))
Sprite(ground, (0, 700))
# add your code here /\  /\  /\


myapp = App()
myapp.run()
'''
###############################################
"""
picture.py
Author: Payton
Credit: Andreas and colorpicker.com

Assignment:

Use the ggame library to "paint" a graphical picture of something (e.g. a house, a face or landscape).

Use at least:
1. Three different Color objects.
2. Ten different Sprite objects.
3. One (or more) RectangleAsset objects.
4. One (or more) CircleAsset objects.
5. One (or more) EllipseAsset objects.
6. One (or more) PolygonAsset objects.

See:
https://github.com/HHS-IntroProgramming/Standards-and-Syllabus/wiki/Displaying-Graphics
for general information on how to use ggame.

See:
http://brythonserver.github.io/ggame/
for detailed information on ggame.

"""
from ggame import App, Color, LineStyle, Sprite, RectangleAsset, CircleAsset, EllipseAsset, PolygonAsset

# add your code here \/  \/  \/
"""
picture.py
Author: <your name here>
Credit: <list sources used, if any>

Assignment:

Use the ggame library to "paint" a graphical picture of something (e.g. a house, a face or landscape).

Use at least:
1. Three different Color objects.
2. Ten different Sprite objects.
3. One (or more) RectangleAsset objects.
4. One (or more) CircleAsset objects.
5. One (or more) EllipseAsset objects.
6. One (or more) PolygonAsset objects.

See:
https://github.com/HHS-IntroProgramming/Standards-and-Syllabus/wiki/Displaying-Graphics
for general information on how to use ggame.

See:
http://brythonserver.github.io/ggame/
for detailed information on ggame.

"""
'''
from ggame import App, Color, LineStyle, Sprite, RectangleAsset, CircleAsset, EllipseAsset, PolygonAsset

# add your code here \/  \/  \/
red = Color(0xff0000, 1.0)
green = Color(0x00ff00, 1.0)
blue = Color(0x0000ff, 1.0)
black = Color(0x000000, 1.0)
teal = Color(0x24B5B5, 1.0)
yellow = Color(0xFFF700, 1.0)
purple = Color(0x6F00FF, 1.0)
orange = Color(0xFF6A00, 1.0)
white = Color(0xFFFFFF, 1.0)

#lines
thinline = LineStyle(1, black)
thickerline = LineStyle(10, green)
greenline = LineStyle(1, green)
redline = LineStyle(1, red)
blueline = LineStyle(1, blue)
tealline = LineStyle(1, teal)
yellowline = LineStyle(1, yellow)
purpleline = LineStyle(1, purple)
orangeline = LineStyle(1, orange)

#shapes
house = RectangleAsset(300, 300, thinline, teal)
grassyhill = CircleAsset(600, greenline, green)
sun = CircleAsset(150, yellowline, yellow)
roof = PolygonAsset([(5,120), (165,1), (335,120)], tealline, purple)
windowa = RectangleAsset(85, 85, thinline, blue)
windowb = RectangleAsset(85, 85, thinline, blue)
windowc = RectangleAsset(85, 85, thinline, blue)
windowd = RectangleAsset(85, 85, thinline, blue)
door = RectangleAsset(70, 90, thinline, red)
knob = CircleAsset(10, thinline, black)
sandyhill = CircleAsset(600, orangeline, orange)
moon = EllipseAsset(150, 150, thinline, white)


#sprites
Sprite(sandyhill, (300, 1100))
Sprite(grassyhill, (680, 1000))
Sprite(house, (520, 240))
Sprite(sun, (1020, 150))
Sprite(roof, (500, 119))
Sprite(windowa, (540, 270))
Sprite(windowb, (720, 270))
Sprite(windowc, (540, 400))
Sprite(windowd, (720, 400))
Sprite(door, (640, 440))
Sprite(knob, (700, 490))
Sprite(moon, (320, 150))
# add your code here /\  /\  /\


myapp = App()
myapp.run()

# add your code here /\  /\  /\


myapp = App()
myapp.run()

'''
##########################################
"""
picture.py
Author: Avery Wallis
Credit: None so far

Assignment:
Use the ggame library to "paint" a graphical picture of something (e.g. a house, a face or landscape).
Use at least:
1. Three different Color objects.
2. Ten different Sprite objects.
3. One (or more) RectangleAsset objects.
4. One (or more) CircleAsset objects.
5. One (or more) EllipseAsset objects.
6. One (or more) PolygonAsset objects.
See:
https://github.com/HHS-IntroProgramming/Standards-and-Syllabus/wiki/Displaying-Graphics
for general information on how to use ggame.
See:
http://brythonserver.github.io/ggame/
for detailed information on ggame.
"""
##################################
'''
from ggame import App, Color, LineStyle, Sprite, RectangleAsset, CircleAsset, EllipseAsset, LineAsset, PolygonAsset


red = Color(0xff0000, 1.0)
green = Color(0x00ff00, 1.0)
blue = Color(0x0000ff, 1.0)
black = Color(0x000000, 1.0)
orange = Color(0xffa500, 1.0)
skin =Color(0xFCD15B, 1.0)
wall=Color(0xE8E8E8, 1.0)
orange=Color(0xFFa500,1.0)
plat=Color(0xB9BDBB,1.0)
gooy=Color(0xCDF238,1.0)
white=Color(0xFFFFFF,1.0)
darkblue=Color(0x052099,1.0)


thinline= LineStyle(1, black)
thickline= LineStyle(5, black)
thickishline= LineStyle(2.5, black)
noline=LineStyle(0, black)
portalline=LineStyle(1, blue)
portalline2=LineStyle(1, orange)


wall=RectangleAsset(500,500, noline, wall)
blueportal=EllipseAsset(27, 60, noline, blue)
orangeportal=EllipseAsset(27, 60, noline, orange)
innerportal=EllipseAsset(24, 57, noline, white)
exit=CircleAsset(70, thinline, plat)
exit2=CircleAsset(20, thinline, plat)
plat=RectangleAsset(250, 50, noline, plat)

# 決定 x 方向的 delta 與 y 方向的 delta, 這裡為各 300
doorline=LineAsset(300, 300, thinline)

goo=PolygonAsset([(0,500),(800,500),(800,600,),(0,600)],noline,gooy)

Sprite(wall, (400,20))
Sprite(wall, (100,20))
Sprite(exit, (800,100))
Sprite(exit2, (800, 100))

# 決定線的起點
line1 = Sprite(doorline, (0, 0))
Sprite(doorline, (10, 100))

line1.x = 100

Sprite(plat, (100,400))
Sprite(plat, (650, 150))
Sprite(orangeportal, (700,90))
Sprite(innerportal, (700,90))
Sprite(blueportal, (200,340))
Sprite(innerportal, (200,340))
Sprite(goo, (100,0))

myapp = App()
myapp.run()

# 請問, 如何畫方格紙上的線, 每個小方塊為 20 x 20 個 pixels
'''

########################################
'''
from ggame import App, Color, LineStyle, Sprite
from ggame import CircleAsset

red = Color(0xff0000, 1.0)
green = Color(0x00ff00, 1.0)
blue = Color(0x0000ff, 1.0)
black = Color(0x000000, 1.0)

thinline = LineStyle(1, black)
mycircle = CircleAsset(5, thinline, blue)
xcoordinates = range(100, 600, 10)

# Generate a list of sprites that form a line!
sprites = [Sprite(mycircle, (x, x*0.5 + 100)) for x in xcoordinates]

myapp = App()
myapp.run()
'''
#################################

"""
multiplication-table.py
Author: Hayden Hatfield
Credit: Mr. Denison
Assignment:

Write and submit a Python program that prints a multiplication table. The user 
must be able to determine the width and height of the table before it is printed.

The final multiplication table should look like this:

Width of multiplication table: 10
Height of multiplication table: 8

  1   2   3   4   5   6   7   8   9  10
  2   4   6   8  10  12  14  16  18  20
  3   6   9  12  15  18  21  24  27  30
  4   8  12  16  20  24  28  32  36  40
  5  10  15  20  25  30  35  40  45  50
  6  12  18  24  30  36  42  48  54  60
  7  14  21  28  35  42  49  56  63  70
  8  16  24  32  40  48  56  64  72  80
"""
Width = input("Width of multiplication table: ")
Height = input("Height of multiplication table: ")
for y in range(1, int(Height)+1 ):
    for x in range(1, int(Width)+1 ):
        print (y*x, end = " ")
        pass
    print()
