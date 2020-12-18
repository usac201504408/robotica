import sim
import numpy as np
import sys
import cv2
import time


def conectar(puerto):
    sim.simxFinish(-1) #Terminar todas las conexiones
    clientID=sim.simxStart('127.0.0.1',puerto,True,True,5000,5) #Iniciar una nueva conexion en el puerto 19999 (direccion por defecto)
    
    if clientID!=-1:
        print ('Conexion establecida')
    
    else:
        sys.exit("Error: no se puede conectar") #Terminar este script
    
    return clientID

def deteccionColor():
    time.sleep(0.9)
    _, camhandle = sim.simxGetObjectHandle(clientID,'Vision_sensor',sim.simx_opmode_oneshot_wait)
    _, resolution, image = sim.simxGetVisionSensorImage(clientID,camhandle,0,sim.simx_opmode_streaming)
    time.sleep(1)

    print("resolution")
    print(resolution)
    print("image")
    print(image)
    print("camhandle")
    print(camhandle)
    img = np.array(image,dtype = np.uint8)
    img.resize([resolution[0],resolution[1],3])
    img = np.rot90(img,2)
    img = np.fliplr(img)
    img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #color rojo
    rojoBajo1 =np.array([0,100,20], np.uint8)
    rojoAlto1 =np.array([8,255,255], np.uint8)
    rojoBajo2 =np.array([175,100,20],np.uint8)
    rojoAlto2 =np.array([179,255,255],np.uint8)
    maskRojo1 = cv2.inRange(hsv, rojoBajo1, rojoAlto1)
    maskRojo2 = cv2.inRange(hsv,rojoBajo2, rojoAlto2)
    maskRed = cv2.add(maskRojo1,maskRojo2)

    #color verde
    verdeBajo = np.array([45,50,50])
    verdeAlto = np.array([80,255,255])
    maskGreen = cv2.inRange(hsv,verdeBajo,verdeAlto)

    #color azul
    azulBajo = np.array([100,65,75])
    azulAlto = np.array([130,255,255])
    maskBlue = cv2.inRange(hsv, azulBajo, azulAlto)

    #buscando el centro de los objetos
    M1 = cv2.moments(maskRed)
    M2 = cv2.moments(maskGreen)
    M3 = cv2.moments(maskBlue)
    area1 = M1['m00']
    area2 = M2['m00']
    area3 = M3['m00']

    if(area1 > 100):
        x = int(M1['m10']/M1['m00'])
        y = int(M1['m01']/M1['m00'])
        cv2.rectangle(img,(x,y),(x+2,y+2),(255,0,0),2)

    if(area2 > 100):
        x = int(M2['m10']/M2['m00'])
        y = int(M2['m01']/M2['m00'])
        cv2.rectangle(img, (x,y), (x+2,y+2),(0,0,255),2)

    if(area3 > 100):
        x = int(M3['m10']/M3['m00'])
        y = int(M3['m01']/M3['m00'])

    #Se muestra la ventana con el objeto detectado
    cv2.imshow('Imagen',img)

def agarrarCubo():
  
    sim.simxSetJointTargetPosition(clientID,armJoints[1], -66.7*np.pi/180 ,sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetPosition(clientID,armJoints[2], 25*np.pi/180 ,sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetPosition(clientID,armJoints[4], -50*np.pi/180 ,sim.simx_opmode_oneshot_wait)

def regresarPosInicial():

    sim.simxSetJointTargetPosition(clientID,armJoints[4], -90*np.pi/180 ,sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetPosition(clientID,armJoints[1], 0 ,sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetPosition(clientID,armJoints[2], 0 ,sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetPosition(clientID,armJoints[0], 0 ,sim.simx_opmode_oneshot_wait)

def colocarCubo1():

    sim.simxSetJointTargetPosition(clientID,armJoints[0], -90*np.pi/180 ,sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetPosition(clientID,armJoints[1], -85*np.pi/180,sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetPosition(clientID,armJoints[2], 60*np.pi/180  ,sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetPosition(clientID,armJoints[4], -70*np.pi/180,sim.simx_opmode_oneshot_wait)

 



clientID = conectar(19999)

#obtener los handlers de todos los joints
codigoRetorno1, irb140 =sim.simxGetObjectHandle(clientID, 'IRB140', sim.simx_opmode_blocking)
codigoRetorno2, tip =sim.simxGetObjectHandle(clientID, 'IRB140_tip', sim.simx_opmode_blocking)
codigoRetorno3, targetSphere =sim.simxGetObjectHandle(clientID, 'IRB140_manipulationSphere', sim.simx_opmode_blocking)
codigoRetorno4, targetSphereBase =sim.simxGetObjectHandle(clientID, 'IRB140_manipulationSphereBase', sim.simx_opmode_blocking)
armJoints=  [-1,-1,-1,-1,-1,-1]
codigoRetorno_j1, armJoints[0] =sim.simxGetObjectHandle(clientID, 'IRB140_joint1', sim.simx_opmode_blocking)
codigoRetorno_j2, armJoints[1] =sim.simxGetObjectHandle(clientID, 'IRB140_joint2', sim.simx_opmode_blocking)
codigoRetorno_j3, armJoints[2] =sim.simxGetObjectHandle(clientID, 'IRB140_joint3', sim.simx_opmode_blocking)
codigoRetorno_j4, armJoints[3] =sim.simxGetObjectHandle(clientID, 'IRB140_joint4', sim.simx_opmode_blocking)
codigoRetorno_j5, armJoints[4] =sim.simxGetObjectHandle(clientID, 'IRB140_joint5', sim.simx_opmode_blocking)
codigoRetorno_j6, armJoints[5] =sim.simxGetObjectHandle(clientID, 'IRB140_joint6', sim.simx_opmode_blocking)
#suctionpad
codigoRetorno_j6, suctionPad =sim.simxGetObjectHandle(clientID, 'suctionPad', sim.simx_opmode_blocking)

print("joints")
print(armJoints)

#maximos de velocidades joints
sim.simxSetJointTargetVelocity(clientID, armJoints[0], 0.1, sim.simx_opmode_oneshot)
sim.simxSetJointTargetVelocity(clientID, armJoints[1], 0.1, sim.simx_opmode_oneshot)
sim.simxSetJointTargetVelocity(clientID, armJoints[2], 0.1, sim.simx_opmode_oneshot)
sim.simxSetJointTargetVelocity(clientID, armJoints[3], 0.1, sim.simx_opmode_oneshot)
sim.simxSetJointTargetVelocity(clientID, armJoints[4], 0.1, sim.simx_opmode_oneshot)


#posiciones iniciales
retorno1, initialTipPosRelative = sim.simxGetObjectPosition(clientID, tip, irb140, sim.simx_opmode_blocking)
movementMode=0
maxJointVelocity=180*np.pi/180 
maxPosVelocity=1.0
maxOrientVelocity=45*np.pi/180

agarrarCubo()
time.sleep(1.5)
sim.simxSetIntegerSignal(clientID, 'activarSuction',1, sim.simx_opmode_oneshot)
time.sleep(1.5)
regresarPosInicial()
time.sleep(1.5)
colocarCubo1()
time.sleep(1.5)
sim.simxSetIntegerSignal(clientID, 'activarSuction',0, sim.simx_opmode_oneshot)
time.sleep(1.5)
regresarPosInicial()



#moviendo los joints de prueba
while(1):
    #sim.simxSetJointPosition(clientID,armJoints[0], 90*np.pi/180 ,sim.simx_opmode_oneshot)
    ##prueba colores


    #Estas lineas es para que la camara quede en el bucle infinito y vuelva a buscar mas cubos
    time.sleep(0.9)
    _, camhandle = sim.simxGetObjectHandle(clientID,'Vision_sensor',sim.simx_opmode_oneshot_wait)
    _, resolution, image = sim.simxGetVisionSensorImage(clientID,camhandle,0,sim.simx_opmode_streaming)
    time.sleep(1)

    _, resolution, image= sim.simxGetVisionSensorImage(clientID, camhandle, 0,sim.simx_opmode_buffer)

    img = np.array(image,dtype = np.uint8)
    img.resize([resolution[0],resolution[1],3])
    img = np.rot90(img,2)
    img = np.fliplr(img)
    img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #color rojo
    rojoBajo1 =np.array([0,100,20], np.uint8)
    rojoAlto1 =np.array([8,255,255], np.uint8)
    rojoBajo2 =np.array([175,100,20],np.uint8)
    rojoAlto2 =np.array([179,255,255],np.uint8)
    maskRojo1 = cv2.inRange(hsv, rojoBajo1, rojoAlto1)
    maskRojo2 = cv2.inRange(hsv,rojoBajo2, rojoAlto2)
    maskRed = cv2.add(maskRojo1,maskRojo2)

    #color verde
    verdeBajo = np.array([45,50,50])
    verdeAlto = np.array([80,255,255])
    maskGreen = cv2.inRange(hsv,verdeBajo,verdeAlto)

    #color azul
    azulBajo = np.array([100,65,75])
    azulAlto = np.array([130,255,255])
    maskBlue = cv2.inRange(hsv, azulBajo, azulAlto)

    #buscando el centro de los objetos
    M1 = cv2.moments(maskRed)
    M2 = cv2.moments(maskGreen)
    M3 = cv2.moments(maskBlue)
    area1 = M1['m00']
    area2 = M2['m00']
    area3 = M3['m00']

    if(area1 > 100):
        x = int(M1['m10']/M1['m00'])
        y = int(M1['m01']/M1['m00'])
        cv2.rectangle(img,(x,y),(x+2,y+2),(255,0,0),2)
        print("es rojo")

    if(area2 > 100):
        x = int(M2['m10']/M2['m00'])
        y = int(M2['m01']/M2['m00'])
        cv2.rectangle(img, (x,y), (x+2,y+2),(0,0,255),2)
        print("es verde")

    if(area3 > 100):
        x = int(M3['m10']/M3['m00'])
        y = int(M3['m01']/M3['m00'])
        print("es azul")

    #idea al mover el robot:
    #hacer un metodo que se llame "agarrarCUbo" y otro "soltar cubo" y que reciban como parametro los angulos
    #enviar los angulos cambiados para cada metodo y cada banda, validando si es rojo, azul, verde 
