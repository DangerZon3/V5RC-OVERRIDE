from vex import *

brain = Brain()
V = AiVision(Ports.PORT1)

V.tag_detection(True)

while True:
    for tag in V.take_snapshot(V.ALL_TAGS):
        print("Tag ID:", tag.id, "X:", tag.centerX, "Y:", tag.centerY)
