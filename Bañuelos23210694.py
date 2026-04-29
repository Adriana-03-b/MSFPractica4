# -*- coding: utf-8 -*-
"""
Created on Fri Apr 24 16:49:30 2026
@author: adrib
"""

"""
Práctica 4: Sistema Endocrino
"""

# Librerías para cálculo numérico y generación de gráficas
import numpy as np
import matplotlib.pyplot as plt
import control as ctrl
import os  # ✅ agregado

# ✅ carpeta donde está el .py
carpeta_actual = os.path.dirname(os.path.abspath(__file__))

x0,t0,tend,dt,w,h = 0,0,15,1e-3,10,5
N = round ((tend-t0)/dt) + 1
t = np.linspace(t0,tend, N)
u = np.ones(N) #step

def Endocrino(R1,R2,C,L):
    num = [R2]
    den = [R1*C*L,L+R1*R2*C,R1+R2]
    sys = ctrl.tf(num,den)
    return sys

#Función de tranferencia: Control
R1,R2,C,L = 1e3,100e3,1e-6,100e-3
syscontrol = Endocrino(R1,R2,C,L)
print (f'Función de transferencia del control (control): {syscontrol}')

#Función de tranferencia: Caso
R1,R2,C,L =  1e3,1e3,1000e-6,100e-3
syscaso = Endocrino(R1,R2,C,L)
print (f'Función de transferencia del caso(caso1): {syscaso}')
##

#Respuestas en lazo abierto
_,Pp0 = ctrl.forced_response(syscontrol,t,u,x0)
_,Pp1 = ctrl.forced_response(syscaso,t,u,x0)

fg1 = plt.figure()
plt.plot(t,Pp0,'-',linewidth=1,color= [0.54,0.52,0.20],label= 'Vs(t): Control')
plt.plot(t,Pp1,'-',linewidth=1,color= [0.66,0.16,0.11],label= 'Vs(t): Caso')
plt.grid(False)
plt.xlim(0,15); plt.xticks(np.arange(0,16,1))
plt.ylim(-0.6,1.4); plt.yticks(np.arange(-0.6-1.6,0.2))
plt.xlabel('t(s)')
plt.ylabel('Vs [V]')
plt.legend(bbox_to_anchor=(0.5,-0.2),loc='center',ncol=3)

# ✅ SOLO se movió guardar antes de show y con ruta correcta
fg1.set_size_inches(w,h)
fg1.tight_layout()
fg1.savefig(os.path.join(carpeta_actual,'Endocrino_lazo_abierto_python.pdf'))

plt.show()

#Controlador PID 
def controlador (kP,kI,kD,sys):
    Cr = 1e-6
    Re = 1/(kI*Cr)
    Rr = kP*Re
    Ce = kD/Rr
    numPID = [Re*Rr*Ce*Cr,(Re*Ce+Rr*Cr),1]
    denPID = [Re*Cr,0]
    PID = ctrl.tf(numPID,denPID)
    X = ctrl.series (PID,sys)
    sysPID = ctrl.feedback (X,1,sign=-1)
    return sysPID

EndoPID = controlador (1315.6501,314901.7711,0.31035,syscaso)
print (f'Función de transferencia del hipotenso en lazo cerrado: {EndoPID}')

#Respuestas del sistema de control en lazo cerrado
_,PID1 = ctrl.forced_response (EndoPID,t,Pp0,x0)

colors = np.array([
    [138, 134, 53],   
    [170, 43, 29],   
    [204, 86, 30]     
]) / 255.0

plt.figure(figsize=(10,6), facecolor='w')
plt.rcParams.update({'font.family':'Times New Roman'})

# Subplot 1: Normotenso vs Hipotenso
plt.subplot(2,1,1)
plt.plot(t, Pp0, '-', color=colors[0], linewidth=1, label=r'$V_s(t):Control$')
plt.plot(t, Pp1, '--', color=colors[1], linewidth=1, label=r'$V_(t):Caso$')
plt.plot(t,PID1,'--',linewidth=1,color= [0.66,0.16,0.11],label= 'PID(t)')
plt.legend(fontsize=10, loc='center', ncol=3, frameon=False,
           bbox_to_anchor=(0.5, 1.25))  
plt.xlabel(r'$t$ [s]', fontsize=11)
plt.ylabel(r'$V_i(t)$ [V]', fontsize=11)
plt.xlim([0,10]); plt.xticks(np.arange(0,11,1))
plt.ylim([0,1.1]); plt.yticks(np.arange(0,1.1,0.2))
plt.title('Control vs Caso', fontsize=10)

plt.tight_layout()

# ✅ SOLO ruta corregida
plt.savefig(os.path.join(carpeta_actual,'Endocrino_PID.pdf'), format='pdf')

plt.show()
  
# Componentes del circuito RLC y función de transferencia
R1,R2,L,C = 1e3,100e3,100e-3,1e-6
num=[R2]
den = [R1*C*L, L+R1*R2*C,R1+R2]
sys = ctrl.tf(num,den)
print(f"Función de transferencia del sistema: {sys}")
print(f"Lambda1, Control: {np.roots(den)[0]}")
print(f"Lambda2 Control: {np.roots(den)[1]}")

# Componentes del circuito RLC y función de transferencia
R1,R2,L,C = 1e3,1e3,100e-3,1000e-6
num=[R2]
den = [R1*C*L, L+R1*R2*C,R1+R2]
sys = ctrl.tf(num,den)
print(f"Función de transferencia del sistema: {sys}")
print(f"Lambda1, Caso: {np.roots(den)[0]}")
print(f"Lambda2 Caso: {np.roots(den)[1]}")
