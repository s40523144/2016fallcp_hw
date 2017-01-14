# week4.py
print("week4.py")
from ggame import App, ImageAsset, Sprite

# Create a displayed object at 100,100 using an image asset
Sprite(ImageAsset("ggame/bunny.jpg"), (0,0))
# Create the app, with a 500x500 pixel stage
app = App(500,500)  
# Run the app
app.run()

