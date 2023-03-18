from cmath import pi, sqrt
from email import iterators
from pickle import FALSE, TRUE
from random import gauss
from subprocess import list2cmdline
from telnetlib import GA
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import webbrowser
import urllib


from time import time
from time import sleep
from urllib.request import urlopen
from matplotlib import style
from PIL import Image , ImageEnhance
from PIL import ImageGrab

kernel_blur = ((1,2,1),(2,8,2),(1,2,1))
kernel_edge_detection_y = ((1,0,-1),(2,0,-2),(1,0,-1))
kernel_edge_detection_x = ((-1,-2,-1),(0,0,0),(1,2,1)) 





#blur gausowski: do wygladzania obrazu(aby lepiej dzialaly inne czesci)
def gaussian_blur(pix_acc_obj,ile):
    pix_val=[0,0,0]
    #print(pix_acc_obj[0,0][0])
    #a=10
    #b=10
    #print(int((pix_acc_obj[a-1,b-1][0]*kernel_blur[0][0])) )
    for i in range(ile):
        for j in range(1,height-2):
            for i in range(1,width-2):
                pix_val[0]=int((pix_acc_obj[i-1,j-1][0]*kernel_blur[0][0]+pix_acc_obj[i,j-1][0]*kernel_blur[1][0]+ pix_acc_obj[i+1,j-1][0]*kernel_blur[2][0]+ pix_acc_obj[i-1,j][0]*kernel_blur[0][1]+ pix_acc_obj[i,j][0]*kernel_blur[1][1]+ pix_acc_obj[i+1,j][0]*kernel_blur[2][1]+ pix_acc_obj[i-1,j+1][0]*kernel_blur[0][2]+ pix_acc_obj[i,j+1][0]*kernel_blur[1][2]+ pix_acc_obj[i+1,j+1][0]*kernel_blur[2][2])/20)
                #print(pix_val)
                pix_val[1]=int((int(pix_acc_obj[i-1,j-1][1])*kernel_blur[0][0]+ int(pix_acc_obj[i,j-1][1])*kernel_blur[1][0]+ int(pix_acc_obj[i+1,j-1][1])*kernel_blur[2][0]+ int(pix_acc_obj[i-1,j][1])*kernel_blur[0][1]+ int(pix_acc_obj[i,j][1])*kernel_blur[1][1]+ int(pix_acc_obj[i+1,j][1])*kernel_blur[2][1]+ int(pix_acc_obj[i-1,j+1][1])*kernel_blur[0][2]+ int(pix_acc_obj[i,j+1][1])*kernel_blur[1][2]+ int(pix_acc_obj[i+1,j+1][1])*kernel_blur[2][2])/20)
                pix_val[2]=int((int(pix_acc_obj[i-1,j-1][2])*kernel_blur[0][0]+ int(pix_acc_obj[i,j-1][2])*kernel_blur[1][0]+ int(pix_acc_obj[i+1,j-1][2])*kernel_blur[2][0]+ int(pix_acc_obj[i-1,j][2])*kernel_blur[0][1]+ int(pix_acc_obj[i,j][2])*kernel_blur[1][1]+ int(pix_acc_obj[i+1,j][2])*kernel_blur[2][1]+ int(pix_acc_obj[i-1,j+1][2])*kernel_blur[0][2]+ int(pix_acc_obj[i,j+1][2])*kernel_blur[1][2]+ int(pix_acc_obj[i+1,j+1][2])*kernel_blur[2][2])/20)
                pix_acc_obj[i,j] = tuple(pix_val)


#wykrywanie krawedzi(self explanatory)
def edge_detection(pix_acc_obj,img):
    img = img.convert("L")
    pix_acc_obj = img.load()
    #img.show()
    temp_x=[]
    temp_y=[]
    is_edge=[]
    
    for j in range(width):
        temp_temp_x=[]
        temp_temp_y=[]
        is_edge_temp=[]
        for i in range(height):
            temp_temp_x.append(0)
            temp_temp_y.append(0)
            is_edge_temp.append(FALSE)
        temp_x.append(temp_temp_x)
        temp_y.append(temp_temp_y)
        is_edge.append(is_edge_temp)

    for j in range(1,height-2):
        for i in range(1,width-2):
            temp_x[i][j]=(pix_acc_obj[i-1,j-1]*kernel_edge_detection_x[0][0])+(pix_acc_obj[i+1,j-1]*kernel_edge_detection_x[2][0])+(pix_acc_obj[i-1,j]*kernel_edge_detection_x[0][1])+(pix_acc_obj[i+1,j]*kernel_edge_detection_x[2][1])+(pix_acc_obj[i-1,j+1]*kernel_edge_detection_x[0][2])+(pix_acc_obj[i+1,j+1]*kernel_edge_detection_x[2][2]) 
            
            temp_y[i][j]=(pix_acc_obj[i-1,j-1]*kernel_edge_detection_y[0][0])+(pix_acc_obj[i,j-1]*kernel_edge_detection_y[1][0])+(pix_acc_obj[i+1,j-1]*kernel_edge_detection_y[2][0])+(pix_acc_obj[i-1,j+1]*kernel_edge_detection_y[0][2])+(pix_acc_obj[i,j+1]*kernel_edge_detection_y[1][2])+(pix_acc_obj[i+1,j+1]*kernel_edge_detection_y[2][2])
    for j in range(1,height-2):
        for i in range(1,width-2):
            if int(sqrt(pow(temp_x[i][j],2)+pow(temp_y[i][j],2) ).real) > 60:
                is_edge[i][j]=1
            else:
                is_edge[i][j]=0
    img = img.convert("HSV")
    return is_edge
     
#tworzenie mapki
def to_map(pix_acc_obj,ile_woda,ile_budynki,ile_laki,ile_lasy,ile_piasek,iterator):
    #PIL uzywa jednostek 0-255 a HSV 0-360 i to jest mniej wiecej mnoznik dla H
    PIL_H_converter = 0.7083333
    #to mnoznik dla S lub V
    PIL_S_V_converter = 2.55
    #print(150 * PIL_H_converter)
    #woda h-150-255 
    
    #aral
    #H_woda_D = int(62 * PIL_H_converter)
    #H_woda_G = int(255 * PIL_H_converter)
    
    #test
    H_woda_D = int(175 * PIL_H_converter)
    H_woda_G = int(255 * PIL_H_converter)
    
    #budynki s<5% i v<5% lub h <36 i >255
    
    #oryginal
    #H_budynki_D = int(255 * PIL_H_converter)
    #H_budynki_G = 255
    #S_budynki = int(10 * PIL_S_V_converter)
    #V_budynki = int(10 * PIL_S_V_converter)

    #test
    H_budynki_D = 255
    H_budynki_G = 255
    S_budynki = int(5 * PIL_S_V_converter)
    V_budynki = int(5 * PIL_S_V_converter)
    
    
    #laki h-62-86
    
    #oryginal
    #H_laki_D = int(62 * PIL_H_converter)
    #H_laki_G = int(86 * PIL_H_converter)

    #test
    H_laki_D = int(62 * PIL_H_converter)
    H_laki_G = int(86 * PIL_H_converter)
    
    
    #lasy h-86-150
    
    #oryginal
    #H_lasy_D = int(86 * PIL_H_converter)
    #H_lasy_G = int(150 * PIL_H_converter)

    #test
    H_lasy_D = int(86 * PIL_H_converter)
    H_lasy_G = int(175 * PIL_H_converter)
    
    #piasek h-36-62 
    #oryginal
    #H_piasek_D = int(0 * PIL_H_converter)
    #H_piasek_G = int(62 * PIL_H_converter)

    #test
    H_piasek_D = 0
    H_piasek_G = int(62 * PIL_H_converter)

    woda_kolor = (int(198 * PIL_H_converter),255,255)
    budynki_kolor = (0,int(3 * PIL_S_V_converter),int(74 * PIL_S_V_converter))
    laki_kolor = (int(135 * PIL_H_converter),int(78*PIL_S_V_converter),int(63*PIL_S_V_converter))
    lasy_kolor = (int(135 * PIL_H_converter),int(79*PIL_S_V_converter),int(36*PIL_S_V_converter))
    piasek_kolor = (int(40 * PIL_H_converter),int(52*PIL_S_V_converter),int(79*PIL_S_V_converter))

    for j in range(height):
        for i in range(width):
            #woda
            if(pix_acc_obj[i,j][0]>H_woda_D and pix_acc_obj[i,j][0]<H_woda_G):
                pix_acc_obj[i,j] = woda_kolor
                ile_woda[iterator] = ile_woda[iterator] + 1
            #budynki
            if((pix_acc_obj[i,j][0]>H_budynki_D and pix_acc_obj[i,j][0]<H_budynki_G) or (pix_acc_obj[i,j][1]<S_budynki or pix_acc_obj[i,j][2]<V_budynki)):
                pix_acc_obj[i,j] = budynki_kolor
                ile_budynki[iterator] = ile_budynki[iterator] +1
            #laki
            if(pix_acc_obj[i,j][0]>H_laki_D and pix_acc_obj[i,j][0]<H_laki_G):
                pix_acc_obj[i,j] = laki_kolor
                ile_laki[iterator] = ile_laki[iterator] + 1
            #lasy
            if(pix_acc_obj[i,j][0]>H_lasy_D and pix_acc_obj[i,j][0]<H_lasy_G):
                pix_acc_obj[i,j] = lasy_kolor
                ile_lasy[iterator] = ile_lasy[iterator] + 1
            #piasek
            if(pix_acc_obj[i,j][0]>H_piasek_D and pix_acc_obj[i,j][0]<H_piasek_G):
                pix_acc_obj[i,j] = piasek_kolor
                ile_piasek[iterator] = ile_piasek[iterator] + 1
    ilewody_t = []
    ilewody = 0
    rows, cols=width,height
    for i in range(rows):
        col = []
        for j in range(cols):
            col.append(0)
        ilewody_t.append(col)
    for j in range(5,height-4):
        for i in range(5,width-4):
            ilewody = 0
            for h in range(j - 5 , j + 5):
                for g in range(i - 5 , i + 5):
                    if pix_acc_obj[g,h]==woda_kolor:
                        ilewody = ilewody + 1
            ilewody_t[i][j] = ilewody
    for j in range(5,height-4):
        for i in range(5,width-4):
           if ilewody_t[i][j]<50 and pix_acc_obj[i,j]==woda_kolor:
               pix_acc_obj[i,j]=lasy_kolor
           if pix_acc_obj[i,j]!=woda_kolor and pix_acc_obj[i,j]!=budynki_kolor and pix_acc_obj[i,j]!=laki_kolor and pix_acc_obj[i,j]!=lasy_kolor and pix_acc_obj[i,j]!=piasek_kolor:
               pix_acc_obj[i,j] = lasy_kolor
           

ile_woda = [0,0]
ile_budynki = [0,0]
ile_laki = [0,0] 
ile_lasy = [0,0] 
ile_piasek = [0,0]
obrazy = [11]   

PIL_H_converter = 0.7083333
PIL_S_V_converter = 2.55

woda_kolor = (int(198 * PIL_H_converter),255,255)
budynki_kolor = (0,int(3 * PIL_S_V_converter),int(74 * PIL_S_V_converter))
laki_kolor = (int(135 * PIL_H_converter),int(78*PIL_S_V_converter),int(63*PIL_S_V_converter))
lasy_kolor = (int(135 * PIL_H_converter),int(79*PIL_S_V_converter),int(36*PIL_S_V_converter))
piasek_kolor = (int(40 * PIL_H_converter),int(52*PIL_S_V_converter),int(79*PIL_S_V_converter))


f = open("text.txt",'a')
for k in range(0,1):
    
    
    img = Image.open(str(obrazy[k])+'-'+str(obrazy[k])+'.jpg')
    img = img.convert("HSV")
    width,height = img.size;
    pixel_access_object = img.load();
    gaussian_blur(pixel_access_object,1)
    print("pierwszy blur")
    is_edge=edge_detection(pixel_access_object,img)
    print("krawedzie")
    gaussian_blur(pixel_access_object,1)
    print("drugi blur")
    to_map(pixel_access_object,ile_woda,ile_budynki,ile_laki,ile_lasy,ile_piasek,k)
    
    print("do mapki")
    for j in range(1,height-2):
            for i in range(1,width-2):
                if(is_edge[i][j]==1):
                    pixel_access_object[i,j]=(0,0,0)
                if pixel_access_object[i,j] == woda_kolor:
                    pixel_access_object[i,j] = budynki_kolor
    gaussian_blur(pixel_access_object,1)
    print("trzeci blur")
    img.show()
    #img = img.convert("RGB")
    #img.save(str(k)+'.jpg')   
    img.close()
    print("")
    
for i in range(0,2):    
    f.write(""+'\n')
    f.write("piksele woda "+str(obrazy[i])+': '+str(ile_woda[i])+'\n')
    f.write("piksele budynki"+str(obrazy[i])+': '+str(ile_budynki[i])+'\n')
    f.write("piksele laki "+str(obrazy[i])+': '+str(ile_laki[i])+'\n')
    f.write("piksele lasy "+str(obrazy[i])+': '+str(ile_lasy[i])+'\n')
    f.write("piksele piasek "+str(obrazy[i])+': '+str(ile_piasek[i])+'\n')
    f.write('\n')

f.write("roznica pomiedzy zdjeciami"+'\n')
f.write(""+'\n')
f.write("woda: " + str(  (ile_woda[1]-ile_woda[0])/(width*height) )+'\n' )
f.write("")
f.write("piasek: " + str( (ile_piasek[1]-ile_piasek[0])/(width*height) )+'\n' )
f.write("")
f.write("lasy: " + str(  (ile_lasy[1]-ile_lasy[0])/(width*height ) )+'\n')
f.write("")
f.write("laki: " + str(  (ile_laki[1]-ile_laki[0])/(width*height) ) +'\n')

#performance time = 1'50"
