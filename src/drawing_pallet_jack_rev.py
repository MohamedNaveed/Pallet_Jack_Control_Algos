import matplotlib.pyplot as plt
import numpy as np
import math

def wrapToPi(theta):
    return math.atan2(math.sin(theta),math.cos(theta))

def dpj(x,y,theta,steer):
    # plotting the center point
    wd = 1.5

    plt.plot(x,y,'go')
    plt.arrow(x,y,2*math.cos(theta),2*math.sin(theta),width=0.05,color='orange')
    # plt.arrow(x,y,math.cos(wrapToPi(theta+steer)),math.sin(wrapToPi(theta+steer)),width=0.01,color='g')
    #wheel cordinates
    A = np.array([-0.3,0.1,1]).T
    B = np.array([0.3,0.1,1]).T
    C = np.array([-0.3,-0.1,1]).T
    D = np.array([0.3,-0.1,1]).T

    #tranform the wheel


    #front cordinates
    a = np.array([0.5,-0.5,1]).T
    b = np.array([0.5,0.5,1]).T
    c = np.array([0,1,1]).T
    d = np.array([0,-1,1]).T
    e = np.array([-0.5,1,1]).T
    f = np.array([-0.5,-1,1]).T

    #dotted front
    X = np.array([-0.5,0.75,1]).T
    Y = np.array([0,0.75,1]).T
    Z = np.array([0.25,0.5,1]).T
    W = np.array([0.25,-0.5,1]).T
    U = np.array([0,-0.75,1]).T
    V = np.array([-0.5,-0.75,1]).T

    #back support
    g = np.array([-1.5,1,1]).T
    h = np.array([-1.5,-1,1]).T
    i = np.array([-1.5,0.75,1]).T
    j = np.array([-1.5,0.25,1]).T
    k = np.array([-1.5,-0.25,1]).T
    l = np.array([-1.5,-0.75,1]).T

    #drawing the pallet_first_ends
    m = np.array([-4,0.75,1]).T
    n = np.array([-4,0.25,1]).T
    o = np.array([-4,-0.25,1]).T
    p = np.array([-4,-0.75,1]).T

    #drawing the lasts
    q = np.array([-4.5,0.75-0.2,1]).T
    r = np.array([-4.5,0.25+0.2,1]).T
    s = np.array([-4.5,-0.25-0.2,1]).T
    t = np.array([-4.5,-0.75+0.2,1]).T

    # Lets flip the image horizontally before rotation
    flip_T = np.array([[-1,0,0],
                       [0,1,0],
                       [0,0,1]])

    #wheel cordinates
    A = np.matmul(flip_T,A)
    B = np.matmul(flip_T,B)
    C = np.matmul(flip_T,C)
    D = np.matmul(flip_T,D)
    #front cordinates
    a = np.matmul(flip_T,a)
    b = np.matmul(flip_T,b)
    c = np.matmul(flip_T,c)
    d = np.matmul(flip_T,d)
    e = np.matmul(flip_T,e)
    f = np.matmul(flip_T,f)

    #dotted front
    X = np.matmul(flip_T,X)
    Y = np.matmul(flip_T,Y)
    Z = np.matmul(flip_T,Z)
    W = np.matmul(flip_T,W)
    U = np.matmul(flip_T,U)
    V = np.matmul(flip_T,V)

    #back support
    g = np.matmul(flip_T,g)
    h = np.matmul(flip_T,h)
    i = np.matmul(flip_T,i)
    j = np.matmul(flip_T,j)
    k = np.matmul(flip_T,k)
    l = np.matmul(flip_T,l)

    #drawing the pallet_first_ends
    m = np.matmul(flip_T,m)
    n = np.matmul(flip_T,n)
    o = np.matmul(flip_T,o)
    p = np.matmul(flip_T,p)

    #drawing the lasts
    q = np.matmul(flip_T,q)
    r = np.matmul(flip_T,r)
    s = np.matmul(flip_T,s)
    t = np.matmul(flip_T,t)

    #Lets also shift the origin
    shift_T = np.array([[1,0,-2.33],
                       [0,1,0],
                       [0,0,1]])

    #wheel cordinates
    A = np.matmul(shift_T,A)
    B = np.matmul(shift_T,B)
    C = np.matmul(shift_T,C)
    D = np.matmul(shift_T,D)
    #front cordinates
    a = np.matmul(shift_T,a)
    b = np.matmul(shift_T,b)
    c = np.matmul(shift_T,c)
    d = np.matmul(shift_T,d)
    e = np.matmul(shift_T,e)
    f = np.matmul(shift_T,f)

    #dotted front
    X = np.matmul(shift_T,X)
    Y = np.matmul(shift_T,Y)
    Z = np.matmul(shift_T,Z)
    W = np.matmul(shift_T,W)
    U = np.matmul(shift_T,U)
    V = np.matmul(shift_T,V)

    #back support
    g = np.matmul(shift_T,g)
    h = np.matmul(shift_T,h)
    i = np.matmul(shift_T,i)
    j = np.matmul(shift_T,j)
    k = np.matmul(shift_T,k)
    l = np.matmul(shift_T,l)

    #drawing the pallet_first_ends
    m = np.matmul(shift_T,m)
    n = np.matmul(shift_T,n)
    o = np.matmul(shift_T,o)
    p = np.matmul(shift_T,p)

    #drawing the lasts
    q = np.matmul(shift_T,q)
    r = np.matmul(shift_T,r)
    s = np.matmul(shift_T,s)
    t = np.matmul(shift_T,t)

    #before rotating the steering wheel


    # A = np.matmul(T_wheel,A)
    # B = np.matmul(T_wheel,B)
    # C = np.matmul(T_wheel,C)
    # D = np.matmul(T_wheel,D)
    # Tranformations for rotation
    T = np.array([[math.cos(wrapToPi(theta)),-math.sin(wrapToPi(theta)),x],
                        [math.sin(wrapToPi(theta)),math.cos(wrapToPi(theta)),y],
                        [0,0,1]])


    #wheel cordinates
    A = np.matmul(T,A)
    B = np.matmul(T,B)
    C = np.matmul(T,C)
    D = np.matmul(T,D)

    # lets find out the rotation point
    x_rot = (A[0] + B[0] + C[0] + D[0])/4
    y_rot = (A[1] + B[1] + C[1] + D[1])/4
    rot_pt = [-x_rot*math.cos(steer) + y_rot*math.sin(steer) + x_rot,-x_rot*math.sin(steer) - y_rot*math.cos(steer) + y_rot]
    #rotate it about the same point for steering angle
    T_wheel = np.array([[math.cos(wrapToPi(steer)),-math.sin(wrapToPi(steer)),rot_pt[0]],
                        [math.sin(wrapToPi(steer)),math.cos(wrapToPi(steer)),rot_pt[1]],
                        [0,0,1]])
    A = np.matmul(T_wheel,A)
    B = np.matmul(T_wheel,B)
    C = np.matmul(T_wheel,C)
    D = np.matmul(T_wheel,D)
    #front cordinates
    a = np.matmul(T,a)
    b = np.matmul(T,b)
    c = np.matmul(T,c)
    d = np.matmul(T,d)
    e = np.matmul(T,e)
    f = np.matmul(T,f)

    #dotted front
    X = np.matmul(T,X)
    Y = np.matmul(T,Y)
    Z = np.matmul(T,Z)
    W = np.matmul(T,W)
    U = np.matmul(T,U)
    V = np.matmul(T,V)

    #back support
    g = np.matmul(T,g)
    h = np.matmul(T,h)
    i = np.matmul(T,i)
    j = np.matmul(T,j)
    k = np.matmul(T,k)
    l = np.matmul(T,l)

    #drawing the pallet_first_ends
    m = np.matmul(T,m)
    n = np.matmul(T,n)
    o = np.matmul(T,o)
    p = np.matmul(T,p)

    #drawing the lasts
    q = np.matmul(T,q)
    r = np.matmul(T,r)
    s = np.matmul(T,s)
    t = np.matmul(T,t)

    back_center = [(n[0]+o[0])/2,(n[1]+o[1])/2]

    #plotting color
    plt.fill([r[0],q[0],m[0],i[0],j[0],n[0],r[0]],[r[1],q[1],m[1],i[1],j[1],n[1],r[1]],color='grey')
    plt.fill([s[0],o[0],k[0],l[0],p[0],t[0],s[0]],[s[1],o[1],k[1],l[1],p[1],t[1],s[1]],color='grey')
    plt.fill([g[0],e[0],f[0],h[0],g[0]],[g[1],e[1],f[1],h[1],g[1]],color='orange')
    plt.fill([e[0],c[0],b[0],a[0],d[0],f[0],e[0]],[e[1],c[1],b[1],a[1],d[1],f[1],e[1]],color='orange')

    plt.fill([A[0],B[0],D[0],C[0],A[0]],[A[1],B[1],D[1],C[1],A[1]],color='blue')


    plt.plot([a[0],b[0]],[a[1],b[1]],'k',linewidth=wd)
    plt.plot([a[0],d[0]],[a[1],d[1]],'k',linewidth=wd)
    plt.plot([c[0],b[0]],[c[1],b[1]],'k',linewidth=wd)
    plt.plot([c[0],e[0]],[c[1],e[1]],'k',linewidth=wd)
    plt.plot([d[0],f[0]],[d[1],f[1]],'k',linewidth=wd)
    plt.plot([e[0],f[0]],[e[1],f[1]],'k',linewidth=wd)

    plt.plot([X[0],Y[0]],[X[1],Y[1]],'g--')
    plt.plot([Z[0],Y[0]],[Z[1],Y[1]],'g--')
    plt.plot([Z[0],W[0]],[Z[1],W[1]],'g--')
    plt.plot([U[0],W[0]],[U[1],W[1]],'g--')
    plt.plot([U[0],V[0]],[U[1],V[1]],'g--')

    plt.plot([g[0],h[0]],[g[1],h[1]],'k',linewidth=wd)
    plt.plot([g[0],e[0]],[g[1],e[1]],'k',linewidth=wd)
    plt.plot([h[0],f[0]],[h[1],f[1]],'k',linewidth=wd)
    plt.plot([i[0],l[0]],[i[1],l[1]],'k',linewidth=wd)

    plt.plot([m[0],i[0]],[m[1],i[1]],'k',linewidth=wd)
    plt.plot([n[0],j[0]],[n[1],j[1]],'k',linewidth=wd)
    plt.plot([o[0],k[0]],[o[1],k[1]],'k',linewidth=wd)
    plt.plot([p[0],l[0]],[p[1],l[1]],'k',linewidth=wd)

    plt.plot([m[0],q[0]],[m[1],q[1]],'k',linewidth=wd)
    plt.plot([q[0],r[0]],[q[1],r[1]],'k',linewidth=wd)
    plt.plot([n[0],r[0]],[n[1],r[1]],'k',linewidth=wd)

    plt.plot([o[0],s[0]],[o[1],s[1]],'k',linewidth=wd)
    plt.plot([s[0],t[0]],[s[1],t[1]],'k',linewidth=wd)
    plt.plot([p[0],t[0]],[p[1],t[1]],'k',linewidth=wd)

    plt.plot([A[0],B[0]],[A[1],B[1]],'k')
    plt.plot([A[0],C[0]],[A[1],C[1]],'k')
    plt.plot([D[0],B[0]],[D[1],B[1]],'k')
    plt.plot([D[0],C[0]],[D[1],C[1]],'k')

    # plt.plot([back_center[0],x],[back_center[1],y],'o--',linewidth=wd)


    # plt.axes().set_aspect('equal','datalim')
    # plt.show()

x = 1
y = 2
theta = math.pi
steer = math.pi/4
# dpj(x,y,theta,steer)
