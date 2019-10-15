import serial
import time
import smtplib
import cv2
import numpy as np
from sdk.api.message import Message
from sdk.exceptions import CoolsmsException

ser = serial.Serial('/dev/ttyACM0', 9600)
#ser = serial.Serial('/dev/rfcomm1', 9600)


TO = "jun7119@naver.com"   # <-- plz regist welfare center's email address
GMAIL_USER = "hcbkahlua@gmail.com"
GMAIL_PASS = "bandkahlua"

def send_email(ret_date, ret_time):a
    SUBJECT = 'USER arrived at HOME!'
    E_MAIL_TEXT = 'User arrived at home safely.\n\n\nDATE : %sArrived Time : %s' %(ret_date, ret_time)
    
    print("Send Email to FAMILY")
    smtpserver = smtplib.SMTP("smtp.gmail.com",587)
    smtpserver.ehlo()
    smtpserver.starttls()
    smtpserver.ehlo
    smtpserver.login(GMAIL_USER, GMAIL_PASS)
    header = 'To:' + TO + '\n' + 'From: ' + GMAIL_USER
    header = header + '\n' + 'Subject:' + SUBJECT + '\n'
    print header
    msg = header + '\n' + E_MAIL_TEXT + ' \n\n'
    smtpserver.sendmail(GMAIL_USER, TO, msg)
    smtpserver.close()

# gps location in message!!
def send_message(token_who, token_state, temp = None):
    #print(token_who, token_temp, temp)
     # set api key, api secret
    api_key = "NCSGDNVN1O3ZUAFC"
    api_secret = "TOTRSYQK2OEZPNGGMJIMIFYW1UT7XWKN"

    ## 4 params(to, from, type, text) are mandatory. must be filled
    params = dict()
    params['type'] = 'sms' # Message type ( sms, lms, mms, ata )
    params['from'] = 'grand papa\'s phone' # Sender number
    
    if token_who == 'FAMILY':
        params['to'] = 'Family phone num' # Recipients Number '01000000000,01000000001'
    
    elif token_who == 'ME':
        params['to'] = 'User phone num' # Recipients Number '01000000000,01000000001'
    '''
    elif token_who == '119':
        params['to'] = '119'
    '''
    if token_state == 'hot':
        params['text'] = 'User\'s temperature is %s \n Have to go to a cool place.' % str(temp)  # Message
        
    elif token_state == 'cold':
        params['text'] = 'User\'s temperature is %s \n Have to go to a warm place.' % str(temp)  # Message
    
    elif token_state == 'down':
        params['text'] = 'User fell down at %s and doesn\'n wake up!!! please take care of him!' #% str() <--- plz input time #  Message
    
    elif token_state == 'emergency':
        params['test'] = 'User hasn\'t moved in 24 hours. PLEASE CHECK HIM OUT!!!'  # Message
        
        
    cool = Message(api_key, api_secret)
    
    try:
        response = cool.send(params)
        print("Success Count : %s" % response['success_count'])
        print("Error Count : %s" % response['error_count'])
        print("Group ID : %s" % response['group_id'])

        if "error_list" in response:
            print("Error List : %s" % response['error_list'])

    except CoolsmsException as e:
        print("Error Code : %s" % e.code)
        print("Error Message : %s" % e.msg)



# rotate function
def Rotate(src, degrees):
    if degrees == 90:
        dst = cv.transpose(src)
        dst = cv.flip(dst, 1)

    elif degrees == 180:
        dst = cv.flip(src, -1)

    elif degrees == 270:
        dst = cv.transpose(src)
        dst = cv.flip(dst, 0)
    else:
        dst = null
    return dst


def motion_dectector():
    # mog2
    fgbg = cv.createBackgroundSubtractorMOG2(varThreshold=100)

                                                                    
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

    #n umber of box
    n_of_box = 0


    #while():#there is person

    time_cnt = 0
    while True: # motion detection
        ret, frame = cap.read()
        fgmask = fgbg.apply(frame)


        '''
        stats : labels information
        centroid : Mat that has label's center of gravity
        '''
    
        _ ,_ ,stats, centroids = cv.connectedComponentsWithStats(fgmask)
    
    
        for index, centroid in enumerate(centroids):
            if stats[index][0] == 0 and stats[index][1] == 0:
                continue
            if np.any(np.isnan(centroid)):
                continue

       
            x, y, width, height, area = stats[index]
            centerX, centerY = int(centroid[0]), int(centroid[1])
        
            # motion detect,, when there is a little movement
            if area > 200:
                cv.circle(frame, (centerX, centerY), 1, (0, 255, 0), 2)
                cv.rectangle(frame, (x, y), (x + width, y + height), (0, 0, 255))
                n_of_box += 1
      
          
        if n_of_box == 0:   #if there is no motion
            time_cnt += 0.5
        
        else:
            time_cnt = 0

        # die
        if time_cnt >= 86400:
            print('USER HAS NO MOTION! Please check his condition...')
            send_message(token_who = "FAMILY", token_temp = 'emergency')
            break
    
        # rotate frame
        fgmask = Rotate(fgmask , 180)
        frame = Rotate(frame , 180)
    
        # monitor_output
        cv.imshow('MOG2_mask',fgmask)
        cv.imshow('Origin_frame',frame)
        print('Number of Box : ', n_of_box)
        print('Time count : ', time_cnt)
   
        # reset 
        n_of_box = 0
    
        # press 'esc' to exit
        k = cv.waitKey(30) & 0xff
        if k == 27:
            break
        
        time.sleep(0.5)

    cap.release()
    cv.destroyAllWindows()



def main():
    
    print("PORT_NAME : " + str(ser.name))         # check which port was really used
    
    
    HOME_ADDR = [0.000000   ,   0.150000 ] #plz input your home [latitude, longitude] position
    
    temperature = []
    gyro_x = []
    gyro_y = []
    gyro_z = []
    gps_latitude = []
    gps_longitude = []
    gps_date = []
    gps_time = []
    
    line = 0
    
    #if line % 3 == 0:
    #print(type(temperature))
    #print(type(temperature[-1]))
    #print(type(ser.readline()))
    threshold = -1500.0
    temp_cnt = 0
    gyro_cnt = 1
    all_cnt = 0
    
    while 1:
        # Every 1 second, data input
        input_line = ser.readline()
        if "start" in input_line:
            temperature.append(float(ser.readline()))
            gyro_x.append(float(ser.readline()))
            gyro_y.append(float(ser.readline()))
            gyro_z.append(float(ser.readline()))
            gps_latitude.append(ser.readline())
            gps_longitude.append(ser.readline())
            gps_date.append(ser.readline())
            gps_time.append(ser.readline())
    
        else:
            continue
            
            
        # Temperature system
        #print(temperature[-1])
        if len(temperature) == 300:   # 5 min was taken, calculate the temperature.
            AVG = sum(temperature) / len(temperature)
            
            if(float(AVG) > 38.5):
                temp_cnt = temp_cnt + 1
                
                send_message(token_who = "ME", token_state = 'hot', temp = AVG) 
                print("Message was sent! It's Too Hot !!! dangerous!!!!! call 119!!!!!") 
                
                if(temp_cnt == 6000):   # 10 min was taken, send to FAMILY.
                    send_message(token_who = "FAMILY", token_state = 'hot', temp = AVG) 
                    print("Message was sent to FAMILY! It's Too Hot !!! dangerous!!!!! call 119!!!!!") 
                    temp_cnt = 0
                
            elif(float(AVG) < 35.0):
                temp_cnt = temp_cnt + 1
                
                send_message(token_who = "ME", token_state = 'cold', temp = AVG) 
                print("Message was sent! It's Too Cold !!! dangerous!!!!! call 119!!!!!") 
                
                if(temp_cnt == 6000):   # 10 min was taken, send to FAMILY.
                    send_message(token_who = "FAMILY", token_state = 'cold', temp = AVG) 
                    print("Message was sent to FAMILY! It's Too Cold !!! dangerous!!!!! call 119!!!!!") 
                    temp_cnt = 0
                    
            temperature = []
        
        
        
        
        
        # gyro system
        
        #print('x =', gyro_x[-1] - 1150)
        #print('y =', gyro_y[-1] - 3050)
        #print('z =', gyro_z[-1] - 1000)
        #print('gyro_cnt =', gyro_cnt)
        #def send_message(token_who, token_state, temp):
        
        if gyro_y[-1] - 3050 < threshold : # first time in danger, check state
            if(len(gyro_y) > 1 and gyro_y[-2] - 3050 < threshold): # if still dangerous state
                gyro_cnt = gyro_cnt + 1
                
                # consist 120 seconds..
                if(gyro_cnt >= 120): 
                    print("Knock Down !!! dangerous!!!!! call 119!!!!!")
                    #send_message(token_who = "FAMILY", token_state = "down")
                    #gyro_cnt = 1
                    
        else:
            gyro_cnt = 0
            
            
            
            
        # gps system
        
    #   gps_latitude[-1]
    #   gps_longitude[-1]
    #   gps_date[-1]
    #   gps_time[-1]
        
        # safety return service
        if HOME_ADDR[0] == gps_latitude[-1] and HOME_ADDR[1] == gps_longitude[-1]:
            print('user has arrived at home')
            # send_email to welfare center
            send_email(gps_date[-1], gps_time[-1])
            # START openCV function to detect user's motion
            motion_dectector()
    
    
    ser.close()             # close port
    
# program on    
main()

