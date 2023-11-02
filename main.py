import sys
import umqtt_robust2 as mqtt
from machine import UART, Pin, ADC, I2C
from time import sleep                        # Alle vores moduler 
from gps_bare_minimum import GPS_Minimum
from neopixel import NeoPixel
import _thread as thread               #Vi omdøber _tread, så det ikke skal skrives hver gang
from mpu6050 import MPU6050


gps_port = 2                               # ESP32 UART port, Educaboard ESP32 default UART port
gps_speed = 9600                           # UART speed, defauls u-blox speed


uart = UART(gps_port, gps_speed)           # UART object creation
gps = GPS_Minimum(uart)                    # GPS object creation

neoled = 12                                # Hvor mange LED'er der er på neopixel
neopin = 26                                # Hvilken Pin.Out port der skal connectes til esp'en

np = NeoPixel(Pin(neopin, Pin.OUT), neoled)     #NeoPixel object creation

neoled2 = 12
neopin2 = 13

np2 = NeoPixel(Pin(neopin2, Pin.OUT), neoled2)  #Neopixel 2's object creation

i2c = I2C(0)

imu = MPU6050(i2c)                              #IMU's object creation


pin_adc_bat = 35                                # Spændingsdeler Pin, så batterimålingen kan laves
bat_scaling = 4.2 / 720                         # The battery voltage divider ratio

bat_adc = ADC(Pin(pin_adc_bat))            # Vores ADC værdi objekt
bat_adc.atten(ADC.ATTN_11DB)               # Dæmbeled så den måler fra 0- til max 3,3V



def read_battery_voltage_avg64():      #Opretter en funktion
    adc_val = 0 # Nulstiller 
    for i in range(64): #Den tager 64 målinger
        adc_val = bat_adc.read() + adc_val     
    voltage = bat_scaling * (adc_val >> 6) # Hurtigere regnemetode end at dividere
    return voltage      # Returnere spændingen

def bat_charge(antal, r, g, b):    
    for i in range(antal):
        np[i] = (r, g, b) # Gør at vi kan tildele både antal pixels der skal tændes og hvilken farve de skal have
        np.write()
        
def speed_int(antal, r, g, b): #Samme funktion under et andet navn for at give overblik
    for i in range(antal):
        np[i] = (r, g, b)
        np.write()

def styrtled(antal, r, g, b):  #Samme funktion under et andet navn for at give overblik
    for i in range(antal):
        np2[i] = (r, g, b)
        np2.write()


batpro = ((read_battery_voltage_avg64() - 3) / (4.2 - 3)*100) #Ligning der udregner ADC værdi til batteriporcent

def get_adafruit_gps():
    speed = lat = lon = None # Opretter variabler med None som værdi
    if gps.receive_nmea_data():
        # hvis der er kommet end bruggbar værdi på alle der skal anvendes
        if gps.get_speed() != -999 and gps.get_latitude() != -999.0 and gps.get_longitude() != -999.0 and gps.get_validity() == "A":
            # gemmer returværdier fra metodekald i variabler
            speed =str(gps.get_speed())
            lat = str(gps.get_latitude())
            lon = str(gps.get_longitude())
            # returnerer data med adafruit gps format
            return speed + "," + lat + "," + lon + "," + "0.0"
        else: # hvis ikke både hastighed, latitude og longtitude er korrekte 
            print(f"GPS data to adafruit not valid:\nspeed: {speed}\nlatitude: {lat}\nlongtitude: {lon}")
            return False
    else:
        return False

def adafruit_data():   #Vi definere vores while løkker, så de kan bruges til threading
    while True:
        try:
            mqtt.web_print(batpro, 'OzzyBush/feeds/ESP32feed') # Printer vores batteriprocent og sender det adafruit, så det kan aflæses
            print(batpro)                                      # Printer procenten i vores shell så vi kan se den
            sleep(4)                                          # Venter 10 sekunder med at sende en til
            
            gps_data = get_adafruit_gps()
            if gps_data: # hvis der er korrekt data så send til adafruit
                print(f'\ngps_data er: {gps_data}') # Så vi kan se data i vores shell
                mqtt.web_print(gps_data, 'OzzyBush/feeds/mapfeed/csv') #Sender gps data til et mapfeed i adafruit
            else:
                print('waiting for GPS data - move GPS to place with access to the sky...') 
            sleep(4)          #Hvis ikke der er forbindelse til satalitten, så giver den en string i vores shell
                               #Igen så giver den data hver 10 sekund pga vores sleep

        
            if len(mqtt.besked) != 0: # Her nulstilles indkommende beskeder
                mqtt.besked = ""            
            mqtt.sync_with_adafruitIO() # igangsæt at sende og modtage data med Adafruit IO             
            print(".", end = '') # printer et punktum til shell, uden et enter        
        

        except KeyboardInterrupt: # Stopper programmet når der trykkes Ctrl + c
            print('Ctrl-C pressed...exiting')
            mqtt.c.disconnect()
            mqtt.sys.exit()

def fart_control():   #Vi definere vores while løkker, så de kan bruges til threading        
    while True:
        if gps.receive_nmea_data():   #Her modtager vi data fra gps_bare_minimum bibloteket, og hvis den er gyldig, så printes værdierne.
            print(f"Speed           : km/t {gps.get_speed()}")
            print(f"Course          : {gps.get_course():.1f}")
 
        if gps.get_speed() > 3:         # Hvis farten er over 30 km/t
            speed_int(12, 0, 10, 0)      # vil neopixel lyse grøn
        if gps.get_speed() > 0 and gps.get_speed() < 3: #hvis den er mellem 0- og 30 km/t
            speed_int(12, 10, 0, 0)                      # vil neopixel lyse rød
        sleep(5)    # Så holder neopixel farven i 10 sekunder
        speed_int(12, 0, 0, 0) # og slukker efterfølgende
        
    
        
        print(batpro) # Printer batteriprocenten 

        if batpro > 90:                    # Bestemmer hvor mange pixels og hvilken farve neopixel skal tænde    
            bat_charge(12, 10, 2, 0)       # når batteriprocenten er bliver 
            sleep(5)                       # Sleep i 5 sekunder, så man kan se resultatet
        elif batpro < 90 and batpro > 80:
            bat_charge(10, 10, 2, 0)
            sleep(5)
        elif batpro < 80 and batpro > 70:
            bat_charge(9, 10, 2, 0)
            sleep(5)
        elif batpro < 70 and batpro > 60:
            bat_charge(8, 10, 2, 0)
            sleep(5)
        elif batpro < 60 and batpro > 50:
            bat_charge(7, 10, 2, 0)
            sleep(5)
        elif batpro < 50 and batpro > 40:
            bat_charge(6, 10, 2, 0)
            sleep(5)
        elif batpro < 40 and batpro > 30:
            bat_charge(5, 10, 2, 0)
            sleep(5)
        elif batpro < 30 and batpro > 20:
            bat_charge(4, 10, 2, 0)
            sleep(5)
        elif batpro < 20 and batpro > 10:
            bat_charge(3, 10, 2, 0)
            sleep(5)
        elif batpro < 10:
            bat_charge(2, 10, 2, 0)
            sleep(5)
        bat_charge(12, 0, 0, 0)              # Slukker efterfølgende de 5 sekunder
            
def styrt():     #Vi definere vores while løkker, så de kan bruges til threading
    n = 0        # Vi laver en variabel der indeholder antallet af styrt - vi skriver 0 for at sikre os det bliver et integer
    styrtled(12, 0, 0, 0) # Starter med at have alle led'er slukket
    while True:
        val = imu.get_values() # får values fra vores mpu6050.py biblotek og kalder det val
        while val["aksy"] > - 20000: # mens gyroscop y er større end -20000
            val = imu.get_values()   # så skal den aftage en måling
            while val["aksy"] < - 20000: # Hvis den er mindre end -20000
                n = n + 1   # Vores tidligere varibel bliver der langt +1 til
                if n > 0: # Der tjekker vi om n er større end 0
                    n2 = n - 1 # Vi laver en variabel der hedder n2 og gemmer værdien af n - 1 for at vi kunne sikre det var det rigtige antal led der bliver tændt - fordi neopixel tæller fra 0 af
                    np2[n2] = (0, 0, 10) # så tildeler vi en farve til den pixel plads den skal være
                    np2.write() # Tænder led og skriver dataen til neopixel
                else:
                    bat_charge(12, 0, 0, 0) # ellers er alt slukket
                sleep(5)
                break
            
            
thread.start_new_thread(adafruit_data,())    # Her ser man vores threads der bliver brugt
thread.start_new_thread(fart_control,())     # til at får hele systemet til at se ud som om
thread.start_new_thread(styrt,())            # det kører på en gang.

