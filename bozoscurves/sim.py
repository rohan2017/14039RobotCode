import pygame
import random
import math

# py-game variables
windowWidth = 1000
windowHeight = 800
pygame.init()
pygame.display.set_caption(str("Quad-copter balance"))
clock = pygame.time.Clock()
black = (0, 0, 0)
screen = pygame.display.set_mode((windowWidth, windowHeight))
screen.fill((255, 255, 255))

loopCount = 0
run = True
userangle = 0
# Fonts and other graphics
myFont = pygame.font.SysFont("Times New Roman", 18)
telemetries = (140, 170, 200, 230, 260, 290, 320, 350, 380, 410)
telem = 0


def addtelemetry(caption, data):
    # this function displays the value of a variable
    global telem
    Label = myFont.render(caption + ":", 1, black)
    display = myFont.render(str(round(data, 4)), 1, black)
    screen.blit(Label, (10, telemetries[telem]))
    screen.blit(display, (len(caption)*8 + 20, telemetries[telem]))
    telem += 1


def handleloop():
    # this handles quitting and also updates the loop counters
    global telem, run, loopCount, mousePressed
    clock.tick(120)
    telem = 0
    loopCount += 1
    if loopCount == 100000:
        loopCount = 0
    screen.fill((255, 255, 255))
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            run = False


def graphpoints(inputlist, color, scale, ypos):
    # Takes in a list and displays it as a line graph
    actualx = 10
    for i in range(0, len(inputlist)-2):
        actualx += 5
        pygame.draw.line(screen, color, (actualx, int(scale * (inputlist[i]) + ypos)), (actualx + 5, int(scale * (inputlist[i+1]) + ypos)), 3)


def sortSpecial(a):

    flip = True
    temp = 0

    while True:
        flip = False
        for i in range(0, len(a)):
            if (a[i + 1] >= 0 and a[i] > a[i + 1]) or (a[i] < 0 and a[i + 1] >= 0):
                flip = True
                temp = a[i]
                a[i] = a[i + 1]
                a[i + 1] = temp
        if not flip:
            break

    return a


def cubicRoots(P):
    a = P[0]
    b = P[1]
    c = P[2]
    d = P[3]
    
    A = b / a
    B = c / a
    C = d / a

    Q, R, D, S, T, Im = 0
    
    Q = (3 * B - math.pow(A, 2)) / 9
    
    R = (9 * A * B - 27 * C - 2 * math.pow(A, 3)) / 54
    D = math.pow(Q, 3) + math.pow(R, 2)
    t = []
    
    if D >= 0:  # complex or duplicate roots
        S = math.copysign(1, (R + math.sqrt(D)) * math.pow(abs(R + math.sqrt(D)), (1 / 3)))
        T = math.copysign(1, (R - math.sqrt(D)) * math.pow(abs(R - math.sqrt(D)), (1 / 3)))
        
        t[0] = -A / 3 + (S + T)  # real root
        t[1] = -A / 3 - (S + T) / 2  # real part of complex root
        t[2] = -A / 3 - (S + T) / 2  # real part of complex root
        Im = abs(math.sqrt(3) * (S - T) / 2);  # complex part of root pair
        
        # discard complex roots
        if Im != 0:
            t[1] = -1
            t[2] = -1
    else:  # distinct real roots
        
        th = math.acos(R / math.sqrt(-math.pow(Q, 3)))

        t[0] = 2 * math.sqrt(-Q) * math.cos(th / 3) - A / 3
        t[1] = 2 * math.sqrt(-Q) * math.cos((th + 2 * math.pi) / 3) - A / 3
        t[2] = 2 * math.sqrt(-Q) * math.cos((th + 4 * math.pi) / 3) - A / 3
        Im = 0.0
    
    # discarded out of spec roots
    for i in range(0,3):
        if t[i] < 0 or t[i] > 1.0:
            t[i]=-1
    
    # sort but place -1 at the end
    t = sortSpecial(t)
    return t


def computeIntersections(px, py, lx, ly):

    X = []
    A = ly[1] - ly[0]  # A = y2 - y1
    B = lx[0] - lx[1]  # B = x1 - x2
    C = lx[0] * (ly[0] - ly[1]) + ly[0] * (lx[1] - lx[0])  # C = x1 * (y1 - y2) + y1 * (x2 - x1)

    bx = bezierCoeffs(px[0], px[1], px[2], px[3])
    by = bezierCoeffs(py[0], py[1], py[2], py[3])
    P = []
    P[0] = A * bx[0] + B * by[0]
    P[1] = A * bx[1] + B * by[1]  # t^2
    P[2] = A * bx[2] + B * by[2]  # t
    P[3] = A * bx[3] + B * by[3] + C  # 1

    r = cubicRoots(P)

    # verify the roots are in bounds of the linear segment
    for (var i=0;i < 3;i++)
        t=r[i];

    X[0]=bx[0] * t * t * t+bx[1] * t * t+bx[2] * t+bx[3];
    X[1]=by[0] * t * t * t+by[1] * t * t+by[2] * t+by[3];

    / * above is intersection point assuming infinitely long line segment,
    make sure we are also in bounds of the line * /
    var s;
    if ((lx[1]-lx[0]) != 0) / * if not vertical line * /
    s=(X[0]-lx[0]) / (lx[1]-lx[0]);
    else
    s=(X[1]-ly[0]) / (ly[1]-ly[0]);

    / * in bounds? * /
    if (t < 0 | | t > 1.0 | | s < 0 | | s > 1.0)
    {
    X[0]=-100; / * move off screen * /
    X[1]=-100;
    }

    / * move intersection point * /
    I[i].setAttributeNS(null, "cx", X[0]);
    I[i].setAttributeNS(null, "cy", X[1]);
    }

    }


while run:
    handleloop()
    
    # Display iables
    pygame.display.flip()