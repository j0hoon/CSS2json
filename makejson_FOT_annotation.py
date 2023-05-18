'''
Annotation이 완료된 데이터를 json 파일로 변환하는 코드입니다.

실행하기 위해 필요한 파일
- SF_PP.mat
- 원본 파일 (ex. CN7_030323_069.bag, RG3_030323_069.mf4)
- mat 파일 (ex. CN7_030323_069.mat)
- Annotation 파일 (ex. Annotation_CN7_030323_069.xlsx)
- Registration 파일 (ex. Registration_CN7_030323_069.xlsx)
- Lane 파일 (ex. Lane_CN7_030323_069.xlsx)

output 
CSS.json
'''

import json
import pandas as pd
import numpy as np
import scipy.io
import string
import scipy.io 
import glob
import os
import pyrosbag as rb
import json
from pathlib import Path
from CSS2json_lib import *
import natsort
from tqdm import tqdm
# SELECT MODE
import mat73
import h5py
from datetime import datetime 


FUSION_TRACK ={
    "ID"                : 25, # ID              [26]
    'REL_POS_Y'         : 26, # REL_POS_Y       [27]
    "REL_POS_X"         : 27, # REL_POS_X       [28]
    "REL_VEL_Y"         : 28, # REL_VEL_Y       [29]
    "REL_VEL_X"         : 29, # REL_VEL_X       [30]
    "HEADING_ANGLE"     : 30, # HEADING_ANGLE   [31]
}

# [모빌아이 파라미터 설정 - PREPROCESSING값 사용]
Lane_data_dict = {'DISTANCE'            : 26, # DISTANCE            [27]
                  'ROAD_SLOPE'          : 27, # ROAD_SLOPE          [28]
                  'CURVATURE'           : 28, # CURVATURE           [29]
                  'CURVATURE_RATE'      : 29, # CURVATURE_RATE      [30]
                  'NEXT_DISTANCE'       : 30, # NEXT_DISTANCE       [31]
                  'NEXT_ROAD_SLOPE'     : 31, # NEXT_ROAD_SLOPE     [32]
                  'NEXT_CURVATURE'      : 32, # NEXT_CURVATURE      [33]
                  'NEXT_CURVATURE_RATE' : 33, # NEXT_CURVATURE_RATE [34]
                  'CONFIDENCE'          : 17} # CONFIDENCE          [18] 

class MakeJson():
    def __init__(self,matDir):
        os.system('cls')
        print("########## MAKE JSON  ##########\n########## CODE START ##########")

        self.matDir = matDir
        self.type = self.CheckType() # RG3 or CN7 or ERROR TYPE
        self.rosbagDir = matDir.replace('Rosbag2Mat','Rosbag')     # original
        self.sfDir = matDir + r'\Perception\SF'
        self.rgDir = matDir + r'\Registration'
        self.manDir = self.matDir + r'\Decision\Maneuver'
        self.date = str(matDir[-6:])

        self.jsonDir = os.getcwd() + r'\json'  # get current working directory
        
        self.adminDataType = 'EXP-' +self.type
        self.adminSampleTime = '0.05'
        self.adminVersion = {
            "CSS":"0.9",
            "LDT":"",
            "SF":""
        }
        self.adminProjectName = " "
        self.adminCMGT = 0
        self.adminAESGT = 8
        self.MakeJsonPath(self.jsonDir, "json")
        self.GeoreferenceType = "Point"
        self.TRACKNUM = 64
        self.FT_ID = 25
        self.FT_RELX = 29

    def AutoCuration(self):
        print("##########  "+str(self.type)+" AUTO  ##########")
        
        if self.type == 'CN7':
            rawDataDir = self.matDir.replace('Rosbag2Mat','Rosbag')
        elif self.type == 'RG3':
            rawDataDir = self.matDir.replace('mat','mf4')
        else :
            raise ("CAR TYPE ERROR!!")

        sfList = natsort.natsorted(glob.glob(self.sfDir + '\\*.mat'))
        matList =  natsort.natsorted(glob.glob(self.matDir + '\\*.mat'))
        if self.type == 'RG3':
            rawDataList = natsort.natsorted(glob.glob(rawDataDir + '\\*.mf4'))
        elif self.type == 'CN7':
            rawDataList = natsort.natsorted(glob.glob(rawDataDir + '\\*.bag'))
        
        man_list = os.listdir(self.manDir)
        
        for _man_path in tqdm(man_list):
            
            fnum = (_man_path.split("\\")[-1]).split("_")[3][:3]
            num = int(fnum) -1
            try:
                sf_file_matching = [tmp_sf_path for tmp_sf_path in sfList if "_" + fnum + "_SF_PP.mat" in tmp_sf_path][0]
                sf_file_fnum = (sf_file_matching.split(".")[-2]).split("_")[-3]
                
                mat_file_matching = [tmp_mat_path for tmp_mat_path in matList if "_" + fnum + ".mat" in tmp_mat_path][0]
                mat_file_fnum = (mat_file_matching.split(".")[-2]).split("_")[-1]
                
                if self.type == "CN7":
                    raw_file_matching = [tmp_mat_path for tmp_mat_path in rawDataList if "_" + fnum + ".bag" in tmp_mat_path][0]
                elif self.type == "RG3" :
                    raw_file_matching = [tmp_mat_path for tmp_mat_path in rawDataList if "_" + fnum + ".mf4" in tmp_mat_path][0]
                else :
                    raise("CAR TYPE ERROR")
                raw_file_fnum = (mat_file_matching.split(".")[-2]).split("_")[-1]
            except:
                continue
            if (fnum == sf_file_fnum and fnum == mat_file_fnum) == False :
                continue
            GPS_STATUS = 0
            CHASSIS_STATUS = 0
            MOBILEYE_STATUS = 0
            FRONT_RADAR_STATUS = 0
            CORNER_RADAR_STATUS = 0
            LIDAR_STATUS = 0
            ODD_STATUS = 0
            CSS_STATUS = [] # (0: Normal, 1: GPS inappropriate, 2: Chassis inappropriate, 3: Mobileye data inappropriate, 4: Front Radar inappropriate, 5: Corner Radar inappropriate, 6: Lidar inappropriate 7: OUT of ODD)
            FRAMESIZE = 0
            
            TYPE = self.type
            DATE = self.date
            sfName = sf_file_matching.split('\\')[-1]
            matName = mat_file_matching.split('\\')[-1]
            rawDataName = raw_file_matching.split('\\')[-1]

            matSf = scipy.io.loadmat(sf_file_matching)
            if self.type == "RG3":
                mat = scipy.io.loadmat(matList[num])
            if self.type == "CN7":
                # mat = mat73.loadmat(matList[num])
                mat = h5py.File(matList[num])
            
            FRAMESIZE = matSf['SF_PP']['sim_time'][0,0].shape[0]
            # FRAMESIZE,CHASSIS_STATUS = self.GetVehicleFrameSize(self.type, mat)
            try:
                registrationFileRoad = pd.read_excel(self.rgDir +r'\Raw\Registration_' + self.type + r'_' + self.date + r'.xlsx',sheet_name = "road")
                registrationFileWrong = pd.read_excel(self.rgDir +r'\Raw\Registration_' + self.type + r'_' + self.date + r'.xlsx',sheet_name = "wrong")
            except:
                raise Exception("Registration.xlsx가 없습니다.")
            
            for tmpidx in range(registrationFileWrong.shape[0]):
                if int(fnum) == registrationFileWrong['dataNum'].iloc[tmpidx]:
                    CSS_STATUS = str(registrationFileWrong['Description'].iloc[tmpidx])

                
            adminDirectoryRaw = rawDataDir + '\\' + rawDataName
            adminDirectoryExported = self.matDir + '\\' + matName
            adminDirectoryRegistration = self.rgDir + '\\Raw\\Registration_' + self.type + '_' + self.date +'.xlsx' 
            time = datetime.fromtimestamp(os.path.getctime(adminDirectoryRaw)).strftime('%Y-%m-%d %H:%M:%S')[-8:]  
            adminDate = "20"+self.date[4:6] +"-"+ self.date[0:2] + "-" + self.date[2:4] +"T" + time
            adminTravelTime = FRAMESIZE*float(self.adminSampleTime)/3600
            adminDirectoryPerceptionLDT = " "
            if self.type == 'CN7' :
                adminDirectoryPerceptionLDT = self.matDir + r'\Perception\LDT\CN7_' +self.date + '_'+ fnum + r'_Lidar_PP.mat'
            adminDirectoryPerceptionSF = self.sfDir + '\\' + self.type + '_' +self.date + '_' + fnum + r'_SF_PP.mat'
            adminDirectoryPerceptionRecognition = self.matDir + r'\Perception\Recognition\Recognition_' + self.type + '_' +self.date + '_' + fnum + r'.xlsx'
            adminDirectoryDecisionManeuver = self.matDir + r'\Decision\Maneuver\Manuever_' + self.type + '_' +self.date + '_' + fnum + r'.xlsx'
            adminFileSize = self.ConverSize(os.path.getsize(adminDirectoryRaw))
            adminGeoreferenceCoordinates,GPS_STATUS = self.GetGeoCoordinates(self.type , mat)
            adminTravelDistance ,CHASSIS_STATUS= self.GetTravelDistance(self.type, mat, float(self.adminSampleTime))
            adminAnnotationType = 'auto'
            #####
            
            driver = {
                "sex":"",
                "age":"",
                "experience":""
            }
            
            
            directory = {
                    "raw":  adminDirectoryRaw.replace("\\\\192.168.75.251","D:\\Shares"),
                    "exported": adminDirectoryExported.replace("\\\\192.168.75.251","D:\\Shares"),
                    "registration":adminDirectoryRegistration.replace("\\\\192.168.75.251","D:\\Shares"),
                    "perception":{
                        "LDT":adminDirectoryPerceptionLDT.replace("\\\\192.168.75.251","D:\\Shares"),
                        "SF":adminDirectoryPerceptionSF.replace("\\\\192.168.75.251","D:\\Shares"),
                        "recognition":adminDirectoryPerceptionRecognition.replace("\\\\192.168.75.251","D:\\Shares")            
                    },
                    "decision":{
                        "maneuver": adminDirectoryDecisionManeuver.replace("\\\\192.168.75.251","D:\\Shares")      
                    }
                }
            #####
            


            lane_width, curvature,MOBILEYE_STATUS = self.GetLaneInfo(self.type,mat)
            if np.size(lane_width) == 0 or np.size(curvature) == 0:
                sceneryLaneWidthMax = float(0.0)
                sceneryLaneWidthMin = np.abs(float(0.0))
                sceneryCurvatureMax = float(0.0)
                sceneryCurvaturemin = float(0.0)
            else :
                sceneryLaneWidthMax = str(round(np.max(lane_width),2))
                sceneryLaneWidthMin = str(np.abs(round(np.min(lane_width),2)))
                sceneryCurvatureMax = str(round(np.max(curvature),8))
                sceneryCurvaturemin = str(round(np.min(curvature),8))

            curveFlag, curveFlagSize = self.CalculateCurveEvent(self.type,matSf)
            sceneryEvent = []
            sceneryEvent = pd.DataFrame(columns=['frameIndex', 'roadGeometry'])
            sceneryEvent = self.get_scenery_event(curveFlag , sceneryEvent)
            sceneryRoadName = self.GetRoadName(registrationFileRoad,num)
            # maneuverFile = self.manDir + '\\' + "Maneuver_" + self.type + '_' +self.date + '_' +fnum + '.xlsx' ################### 여기 는 나중에 바뀔 내용
            # maneuverFile = os.getcwd() + '\\Output_xlsx\\'+self.type +'_' + self.date+'\\Annotation_'+self.type+'_'+self.date+'_' + fnum +'.xlsx'
            # maneuverFile = os.getcwd() + '\\Output_xlsx\\'+self.type +'_' + self.date+'\\Maneuver_'+self.type+'_'+self.date+'_' + fnum +'.xlsx'
            maneuverFile = self.manDir +'\\Maneuver_'+self.type+'_'+self.date+'_' + fnum +'.xlsx'
            try:
                maneuverLabel = pd.read_excel(maneuverFile, sheet_name = 'Label')
            except:
                try:
                    maneuverLabel = pd.read_excel(maneuverFile)
                except:
                    continue
            label = maneuverLabel

            # sceneryRoadLabel = label.iloc[:,6:8].query('RoadFrameIndex >= 0')
            # sceneryRoadLabelNum =  sceneryRoadLabel.shape[0]   


            illumination = int(time[:2])
            if (illumination >= 6 and illumination < 18):
                illumination = "Day"
            else:
                illumination = "Night"
            environmentIllumination = ["illumination" , illumination]
            environmentWeather = ["weather" , " "]              


            dynamicInitEgoVelocity = (np.array((matSf['SF_PP']['In_Vehicle_Sensor_sim'][0,0][0,7])) +np.array((matSf['SF_PP']['In_Vehicle_Sensor_sim'][0,0][0,6])))/2
            dynamicInitRelVelX = np.array((matSf['SF_PP']['Fusion_Track_Maneuver'][0,0][25,0,0]))      
            maneuverLabelDynamic = label.iloc[:,0:5]    
            indexSize = maneuverLabelDynamic.dropna().shape[0]
            columnSize = maneuverLabelDynamic.dropna().shape[1]
            initCount = int(np.size(maneuverLabelDynamic.query('FrameIndex == 1'))/columnSize)

            # tmpEgoVelocity = 0
            # tmpRelvelX = 0
            
            tmp_long_pos = 0
            tmp_lat_pos = 0
            tmp_long_vel = 0
            tmp_lat_vel = 0
            tmp_long_acc = 0
            tmp_lat_acc = 0
            
            
            last_frameIndex = int(label['FrameIndex'].iloc[indexSize-1])
            
            longitudinalPosition = np.zeros(indexSize)
            longitudinalActionVelocity = np.zeros(indexSize)
            longitudinalActionAcceleration = np.zeros(indexSize)
            lateralPosition = np.zeros(indexSize)
            lateralActionVelocity = np.zeros(indexSize)
            lateralActionAcceleration = np.zeros(indexSize)
            

            for frameIndex in range(indexSize):
                frameNum = int(label['FrameIndex'].iloc[frameIndex]) -1 ## python 은 -1 되어서 사용해야 하므로 전처리
                
                ### 최대 프레임을 넘지 않는지 체크
                if frameNum +1 > FRAMESIZE:
                    break
                
                try:
                    # tmpEgoVelocity = (float(matSf['SF_PP']['In_Vehicle_Sensor_sim'][0,0][frameNum , 6]) + float(matSf['SF_PP']['In_Vehicle_Sensor_sim'][0,0][frameNum , 7]))/2
                    for trackIdx in range(self.TRACKNUM):
                        # if int(matSf['SF_PP']['Fusion_Track_Maneuver'][0,0][FUSION_TRACK['ID'],trackIdx,frameNum]) == int(label['ID'].iloc[0]):  # int(label['ID'].iloc[0]) 을 하는이유 Ego 의 아이디와 비교해서 확인해야 하기 때문에
                        if int(matSf['SF_PP']['Fusion_Track_Maneuver'][0,0][FUSION_TRACK['ID'],trackIdx,frameNum]) == int(label['ID'].iloc[0]):  # int(label['ID'].iloc[0]) 을 하는이유 Ego 의 아이디와 비교해서 확인해야 하기 때문에                            
                            
                            
                            tmp_long_pos = matSf['SF_PP']['Fusion_Track_Maneuver'][0,0][FUSION_TRACK['REL_POS_Y'],trackIdx,frameNum]
                            tmp_lat_pos = matSf['SF_PP']['Fusion_Track_Maneuver'][0,0][FUSION_TRACK['REL_POS_X'],trackIdx,frameNum]
                            tmp_long_vel = matSf['SF_PP']['Fusion_Track_Maneuver'][0,0][FUSION_TRACK['REL_VEL_Y'],trackIdx,frameNum]
                            tmp_lat_vel = matSf['SF_PP']['Fusion_Track_Maneuver'][0,0][FUSION_TRACK['REL_VEL_X'],trackIdx,frameNum]
                            tmp_long_acc = matSf['SF_PP']['Fusion_Track_Maneuver'][0,0][FUSION_TRACK['REL_ACC_Y'],trackIdx,frameNum]
                            tmp_lat_acc = matSf['SF_PP']['Fusion_Track_Maneuver'][0,0][FUSION_TRACK['REL_ACC_X'],trackIdx,frameNum]
                            
                    
                    longitudinalPosition[frameIndex] = float(tmp_long_pos)
                    longitudinalActionVelocity[frameIndex] =float(tmp_long_vel)
                    longitudinalActionAcceleration[frameIndex] = float(tmp_long_acc)
                    lateralPosition[frameIndex] = float(tmp_lat_pos)
                    lateralActionAcceleration[frameIndex] = float(tmp_lat_acc)
                    lateralActionVelocity[frameIndex] = float(tmp_lat_vel)                        
                except:
                    continue
                



            for frameIndex in range(indexSize):
                frameNum = int(label['FrameIndex'].iloc[frameIndex]) -1 ## python 은 -1 되어서 사용해야 하므로 전처리
                
                ### 최대 프레임을 넘지 않는지 체크
                if frameNum +1 > FRAMESIZE:
                    break
                
                try:
                    # tmpEgoVelocity = (float(matSf['SF_PP']['In_Vehicle_Sensor_sim'][0,0][frameNum , 6]) + float(matSf['SF_PP']['In_Vehicle_Sensor_sim'][0,0][frameNum , 7]))/2
                    for trackIdx in range(self.TRACKNUM):
                        # if int(matSf['SF_PP']['Fusion_Track_Maneuver'][0,0][FUSION_TRACK['ID'],trackIdx,frameNum]) == int(label['ID'].iloc[0]):  # int(label['ID'].iloc[0]) 을 하는이유 Ego 의 아이디와 비교해서 확인해야 하기 때문에
                        if int(matSf['SF_PP']['Fusion_Track_Maneuver'][0,0][FUSION_TRACK['ID'],trackIdx,frameNum]) == int(label['ID'].iloc[frameIndex]):  
                            
                            
                            tmp_long_pos = matSf['SF_PP']['Fusion_Track_Maneuver'][0,0][FUSION_TRACK['REL_POS_Y'],trackIdx,frameNum]
                            tmp_lat_pos = matSf['SF_PP']['Fusion_Track_Maneuver'][0,0][FUSION_TRACK['REL_POS_X'],trackIdx,frameNum]
                            tmp_long_vel = matSf['SF_PP']['Fusion_Track_Maneuver'][0,0][FUSION_TRACK['REL_VEL_Y'],trackIdx,frameNum]
                            tmp_lat_vel = matSf['SF_PP']['Fusion_Track_Maneuver'][0,0][FUSION_TRACK['REL_VEL_X'],trackIdx,frameNum]
                            tmp_long_acc = matSf['SF_PP']['Fusion_Track_Maneuver'][0,0][FUSION_TRACK['REL_ACC_Y'],trackIdx,frameNum]
                            tmp_lat_acc = matSf['SF_PP']['Fusion_Track_Maneuver'][0,0][FUSION_TRACK['REL_ACC_X'],trackIdx,frameNum]
                            
                    
                    longitudinalPosition[frameIndex] = float(tmp_long_pos)
                    longitudinalActionVelocity[frameIndex] =float(tmp_long_vel)
                    longitudinalActionAcceleration[frameIndex] = float(tmp_long_acc)
                    lateralPosition[frameIndex] = float(tmp_lat_pos)
                    lateralActionAcceleration[frameIndex] = float(tmp_lat_acc)
                    lateralActionVelocity[frameIndex] = float(tmp_lat_vel)                        
                except:
                    continue
                
















            #########################################################################################################################
            #participant
            #########################################################################################################################
            participant = []
            for i in range(indexSize):
                # participant_objframe['participants'].append(participants)
                participants = self.GetParticipants(matSf,i,maneuverLabelDynamic,FRAMESIZE)
                
                participant.append(self.GetParticipantObjframe(i,participants,maneuverLabelDynamic))      
            numInitLane = ""
            numMaxLane =""
            scenery = {
                "roadName":sceneryRoadName,
                "event":[],
                "laneWidthMax":sceneryLaneWidthMax,
                "laneWidthMin":sceneryLaneWidthMin,
                "curvatureMax":sceneryCurvatureMax ,
                "curvatureMin":sceneryCurvaturemin,
                "numInitLane":numInitLane,
                "numMaxLane":numMaxLane
            }
            curveFlagSize
    
            # if mode == AUTOCALCULATIONMODE :
            for i in range(curveFlagSize):
                scenery['event'].append({"frameIndex":int(sceneryEvent['frameIndex'].iloc[i]),"roadGeometry":sceneryEvent['roadGeometry'].iloc[i]})
            # elif mode == ANNOTATIONMODE  :
            #     for i in range(sceneryRoadLabelNum):
            #         scenery['event'].append({"frameIndex":int(sceneryRoadLabel['RoadFrameIndex'].iloc[i]),"roadGeometry":sceneryRoadLabel['roadGeometry'].iloc[i]})
        
            # #이하 기능은 annotation이 2줄 이상 있을시에는 annotation을 이용하고 그 외에는 자동으로 생성하는 것을 사용하는 기능입니다. 
            # elif mode == MIXMODE  :
            #     if sceneryRoadLabelNum == 1:
            #         for i in range(curveFlagSize):
            #             scenery['event'].append({"frameIndex":int(sceneryEvent['frameIndex'].iloc[i]),"roadGeometry":sceneryEvent['roadGeometry'].iloc[i]})
            #     else :
            #         for i in range(sceneryRoadLabelNum):
            #             scenery['event'].append({"frameIndex":int(sceneryRoadLabel['RoadFrameIndex'].iloc[i]),"roadGeometry":sceneryRoadLabel['roadGeometry'].iloc[i]})
            # else :
            #     print("select correct mode!!")     
    
            environment = {
                    environmentIllumination[0]:environmentIllumination[1],
                    environmentWeather[0]:environmentWeather[1]
            }        


            dynamic = {
                "init":[],
                "story":{
                    "event":[]
                }
            }          
            
            dynamic_init_action_ego = { # for init 
                "longitudinalAction":{     
                    "velocity":float(longitudinalActionVelocity[0]), # ego 의 차속
                    "acceleration":float(longitudinalActionAcceleration[0])
                                },
                "lateralAction":{
                    "acceleration":float(lateralActionAcceleration[0]),
                    "velocity":float(lateralActionVelocity[0])
                } 
            }

            dynamic_init_action = { # for init 이지만 ego 가 아닌 대상을 위해
                "longitudinalAction":{     
                    "velocity":" ",
                    "acceleration":longitudinalActionAcceleration
                                },
                "lateralAction":{
                    "acceleration":lateralActionAcceleration,
                    "velocity":lateralActionVelocity
                } 
            }  

            for i in range(initCount):
                if i ==0:
                    dynamic['init'].append({"frameIndex":int(maneuverLabelDynamic['FrameIndex'].iloc[i]),\
                    "participantID":int(maneuverLabelDynamic['ID'].iloc[i]),"recognition":str(maneuverLabelDynamic['Recognition'].iloc[i])\
                    ,"maneuver":str(maneuverLabelDynamic['Maneuver'].iloc[i]),"category":int(maneuverLabelDynamic['Category'].iloc[i]),\
                    "action": dynamic_init_action_ego})
                else :
                    dynamic['init'].append({"frameIndex":int(maneuverLabelDynamic['FrameIndex'].iloc[i]),\
                    "participantID":int(maneuverLabelDynamic['ID'].iloc[i]),"recognition":str(maneuverLabelDynamic['Recognition'].iloc[i])\
                    ,"maneuver":str(maneuverLabelDynamic['Maneuver'].iloc[i]),"category":int(maneuverLabelDynamic['Category'].iloc[i]),\
                    "action": dynamic_init_action})                   

            dynamic_story_startTrigger = {
                "conditionName":" ",
                "participantID":" "
            }

            for i in range(initCount , indexSize): # init 이 아닌 부분을 전부 반복문으로 실행
                dynamic_event_key ={
                    "frameIndex":int(maneuverLabelDynamic['FrameIndex'].iloc[i]),
                    "startTrigger":dynamic_story_startTrigger,
                    "actors":{
                        "participantID":int(maneuverLabelDynamic['ID'].iloc[i]),
                        "category":int(maneuverLabelDynamic['Category'].iloc[i]),
                        "recognition":str(maneuverLabelDynamic['Recognition'].iloc[i]),
                        "maneuver":str(maneuverLabelDynamic['Maneuver'].iloc[i])
                    },
                    "action":{
                        "longitudinalAction":{     
                            "position":longitudinalPosition[i],
                            "velocity":longitudinalActionVelocity[i],
                            "acceleration":longitudinalActionAcceleration[i]
                                        },
                        "lateralAction":{
                            "position":lateralPosition[i],
                            "acceleration":lateralActionAcceleration[i],
                            "velocity":lateralActionVelocity[i]
                        }                     
                    }
                }
                dynamic['story']['event'].append(dynamic_event_key)

            ### Generate CSS ###

            CSS_STATUS = self.CheckCSSStatus(GPS_STATUS,CHASSIS_STATUS,MOBILEYE_STATUS,FRONT_RADAR_STATUS,CORNER_RADAR_STATUS,LIDAR_STATUS,ODD_STATUS,CSS_STATUS = CSS_STATUS)
            adminStatus = CSS_STATUS
            CSS_STATUS = []        
            
            CSS = self.GetCSS(self.adminDataType,self.adminSampleTime,self.adminVersion,self.adminProjectName,directory,driver,\
                adminDate,adminTravelTime,adminFileSize,self.GeoreferenceType,adminGeoreferenceCoordinates,self.adminCMGT,\
                self.adminAESGT,adminStatus,adminTravelDistance,adminAnnotationType,scenery,environment,dynamic,participant)      
            # /print(CSS) 
            jsonFilePath = self.jsonDir +"\\"+ TYPE + '_' + DATE + "_AutoCuration"
            jsonFileName = str(num+1).zfill(3) + r'_AutoCuration_temp.json'
            if os.path.isdir(self.jsonDir) == False:
                os.mkdir(self.jsonDir)
            if os.path.isdir(jsonFilePath) == False:
                os.mkdir(jsonFilePath)            
            with open(jsonFilePath+jsonFileName, 'w') as outfile:
                json.dump(CSS, outfile,indent='\t')    

        jsonList = natsort.natsorted(glob.glob(self.jsonDir + '\\*_AutoCuration_temp.json'))
        temp = []
        for i in range(np.size(jsonList)):
            with open(jsonList[i]) as file:
                data = json.load(file)
                temp.append(data)
        realTime = datetime.now()
        realTime = '_'+str(realTime)[:4] +'_'+str(realTime)[5:7] +'_'+ str(realTime)[8:10] +'_'+ str(realTime)[11:13]+'_'+str(realTime)[14:16]
        

        temp_num = 0
        with open(jsonFilePath + '\\' + TYPE+ '_' + DATE+'_'+realTime+'_AutoCuration.json' ,"w") as new_file:
            # json.dump(temp , new_file,indent='\t')
            json.dump([temp[i][0] for i in range(np.size(jsonList))] , new_file,indent='\t')
        
        
        for file in jsonList:
            if file.endswith('_AutoCuration_temp.json'):
                os.remove(file)
        print("########## CODE " +self.type + "_" + "AUTOCURATION END   ##########")
    
    def CheckType(self):
        if self.matDir.split('\\\\192.168.75.251\\Shares')[1][5] == 'G':
            return 'RG3'
        elif self.matDir.split('\\\\192.168.75.251\\Shares')[1][5] == 'A':
            return 'CN7'
        else:
            raise Exception("부적절한 경로 입력입니다.")

    def MakeJsonPath(self, dir , name):
        if os.path.isdir(dir) == False: 
            os.mkdir(dir)
            print([name + " path generated!"])
    
    def ConverSize(self, size_bytes):
        import math
        """_summary_
        Args:
            size_bytes (_type_): _description_

        Returns:
            _type_: _description_
        """
        if size_bytes == 0:
            return "0B"

        # size_name = ("B", "KB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB")
        i = int(math.floor(math.log(size_bytes, 1000)))
        
        p = math.pow(1024, i)
        s = round(size_bytes /p, 8)
        # s = round(size_bytes / p, 10)
        # return "%s %s" % (s, size_name[i])
        return s    
    
    def GetGeoCoordinates(self, TYPE ,Mat):
        GPS_STATUS = 0
        if TYPE == 'RG3':
            try:
                admin_georeference_coordinates = [Mat['Longitude_decimal_Xsens'][0][0] + Mat['Longitude_integer_Xsens'][0][0],Mat['Latitude_decimal_Xsens'][0][0]+Mat['Latitude_integer_Xsens'][0][0]]
            except:
                GPS_STATUS = 1
                admin_georeference_coordinates = [0,0]      
        
        elif TYPE == 'CN7':
            try:
                # gnss_filter_positionlla_vector =pd.DataFrame(np.array(Mat['GNSS']['Filter_Positionlla_Vector']))
                gnss_lat = float(Mat['GNSS']['GNSS_Latitude'][0][0])
                gnss_long = float(Mat['GNSS']['GNSS_Longitude'][0][0])
                admin_georeference_coordinates = [round(gnss_long,8),round(gnss_lat,8)]
                # admin_georeference_coordinates=[round(float(gnss_filter_positionlla_vector[0].loc[0]['X'][0]),8),round(float(gnss_filter_positionlla_vector[0].loc[0]['Y'][0] ),8)]
            except:
                GPS_STATUS = 1
                admin_georeference_coordinates = [0,0]      
        return admin_georeference_coordinates, GPS_STATUS
    
    def GetTravelDistance(self, TYPE, Mat,admin_sampleTime):
        CHASSIS_STATUS = 0
        if TYPE == 'RG3':
            try:
                admin_travelDistance = round(((pd.DataFrame(Mat['WHL_SpdRLVal']) + pd.DataFrame(Mat['WHL_SpdRRVal']))/2).sum()[0]/3600*float(admin_sampleTime),8)
            except:
                CHASSIS_STATUS = 1
                admin_travelDistance = 0
        
        elif TYPE == 'CN7':
            chassis_data = pd.DataFrame(columns=['mean_spd'])
            chassis_data['mean_spd'] = (Mat['Chassis']['LOG_BYTE0_WHLSPDRL'][0,:] + Mat['Chassis']['LOG_BYTE0_WHLSPDRR'][0,:])/2
            try:
                admin_travelDistance = round((chassis_data['mean_spd'].sum()/3600)*admin_sampleTime,8)
            except:
                CHASSIS_STATUS = 1
                admin_travelDistance = 0
        return admin_travelDistance, CHASSIS_STATUS

    def GetLaneInfo(self, TYPE,Mat):
        MOBILEYE_STATUS =0
        lane_width = []
        curvature = []
        if TYPE == 'RG3':
            try:
                FR_CMR_Ln_LftDptDstVal = Mat['FR_CMR_Ln_LftDptDstVal']

            except:
                MOBILEYE_STATUS =1
                lane_width = 0
                curvature=0
                return lane_width, curvature ,  MOBILEYE_STATUS
            
            FR_CMR_Ln_LftDptDstVal = Mat['FR_CMR_Ln_LftDptDstVal']
            FR_CMR_Ln_LftCurveVal = Mat['FR_CMR_Ln_LftCurveVal']
            FR_CMR_Ln_QualLvlLft01Sta = Mat['FR_CMR_Ln_QualLvlLft01Sta']
            FR_CMR_Ln_QualLvlRt01Sta = Mat['FR_CMR_Ln_QualLvlRt01Sta']
            FR_CMR_Ln_RtDptDstVal = Mat['FR_CMR_Ln_RtDptDstVal']
            FR_CMR_Ln_RtCurveVal = Mat['FR_CMR_Ln_RtCurveVal']
        
        elif TYPE == "CN7" :
            try:
                FR_CMR_Ln_LftDptDstVal = Mat['Mobileye_Lane']['ME_Left_Lane_A_QualityLhME'][0,:]

            except:
                MOBILEYE_STATUS =1
                lane_width = 0
                curvature=0
                return lane_width, curvature ,  MOBILEYE_STATUS
            
            FR_CMR_Ln_LftDptDstVal = Mat['Mobileye_Lane']['ME_Left_Lane_A_QualityLhME'][0,:]
            FR_CMR_Ln_LftCurveVal = Mat['Mobileye_Lane']['ME_Right_Lane_A_QualityRhME'][0,:]
            FR_CMR_Ln_QualLvlLft01Sta = Mat['Mobileye_Lane']['ME_Left_Lane_A_LaneMarkPositionC0LhME'][0,:]
            FR_CMR_Ln_QualLvlRt01Sta = Mat['Mobileye_Lane']['ME_Right_Lane_A_LaneMarkPositionC0RhME'][0,:]
            FR_CMR_Ln_RtDptDstVal = Mat['Mobileye_Lane']['ME_Left_Lane_A_LaneMarkModelAC2LhME'][0,:]
            FR_CMR_Ln_RtCurveVal = Mat['Mobileye_Lane']['ME_Right_Lane_A_LaneMarkModelAC2RhME'][0,:]


        for me_idx in range(np.size(FR_CMR_Ln_LftDptDstVal)):
            if (FR_CMR_Ln_QualLvlLft01Sta[me_idx] ==3 and FR_CMR_Ln_QualLvlRt01Sta[me_idx] == 3):
                lane_width.append(-FR_CMR_Ln_LftDptDstVal[me_idx]+FR_CMR_Ln_RtDptDstVal[me_idx])
                curvature.append((FR_CMR_Ln_LftCurveVal[me_idx]+FR_CMR_Ln_RtCurveVal[me_idx])/2)
            elif FR_CMR_Ln_QualLvlLft01Sta[me_idx] ==3:
                lane_width.append(-FR_CMR_Ln_LftDptDstVal[me_idx]*2)
                curvature.append(FR_CMR_Ln_LftCurveVal[me_idx])
            elif FR_CMR_Ln_QualLvlRt01Sta[me_idx] == 3:
                lane_width.append(FR_CMR_Ln_RtDptDstVal[me_idx]*2)
                curvature.append(FR_CMR_Ln_RtCurveVal[me_idx])
            elif (FR_CMR_Ln_QualLvlLft01Sta[me_idx] ==2 and FR_CMR_Ln_QualLvlRt01Sta[me_idx] == 2):
                lane_width.append(-FR_CMR_Ln_LftDptDstVal[me_idx]+FR_CMR_Ln_RtDptDstVal[me_idx])
                curvature.append((FR_CMR_Ln_LftCurveVal[me_idx]+FR_CMR_Ln_RtCurveVal[me_idx])/2)
            elif FR_CMR_Ln_QualLvlLft01Sta[me_idx] ==2:
                lane_width.append(-FR_CMR_Ln_LftDptDstVal[me_idx]*2)
                curvature.append(FR_CMR_Ln_LftCurveVal[me_idx])
            elif FR_CMR_Ln_QualLvlRt01Sta[me_idx] == 2:
                lane_width.append(FR_CMR_Ln_RtDptDstVal[me_idx]*2)
                curvature.append(FR_CMR_Ln_RtCurveVal[me_idx])
        
        return lane_width, curvature,MOBILEYE_STATUS   

    def CalculateCurveEvent(self,TYPE ,Matsf):
        CURVATURE = (np.array(Matsf['SF_PP']['FRONT_VISION_LANE'][0,0]['PREPROCESSING'][0,0]['CURVATURE'][0,0][0,0]))
        LEFT_LANE = (np.array(Matsf['SF_PP']['FRONT_VISION_LANE'][0,0]['LEFT_LANE'][0,0][0,0]))
        Front_Vision_Lane_sim = np.array((Matsf['SF_PP']['Front_Vision_Lane_sim'][0,0]))
        sim_time = pd.DataFrame(np.array(Matsf['SF_PP']['sim_time']))[0][0]
        # if TYPE == 'RG3':
        #     CURVATURE = (np.array(Matsf['SF_PP']['FRONT_VISION_LANE'][0,0]['PREPROCESSING'][0,0]['CURVATURE'][0,0][0,0]))
        #     LEFT_LANE = (np.array(Matsf['SF_PP']['FRONT_VISION_LANE'][0,0]['LEFT_LANE'][0,0][0,0]))
        #     Front_Vision_Lane_sim = np.array((Matsf['SF_PP']['Front_Vision_Lane_sim'][0,0]))
        #     sim_time = pd.DataFrame(np.array(Matsf['SF_PP']['sim_time']))[0][0]
        
        # elif TYPE == 'CN7':
        #     CURVATURE = (np.array(Matsf['SF_PP']['FRONT_VISION_LANE'][0,0]['PREPROCESSING'][0,0]['CURVATURE'][0,0][0,0]))
        #     LEFT_LANE = (np.array(Matsf['SF_PP']['FRONT_VISION_LANE'][0,0]['LEFT_LANE'][0,0][0,0]))
        #     Front_Vision_Lane_sim =  np.array((Matsf['SF_PP']['Front_Vision_Lane_sim'][0,0]))
        #     sim_time = pd.DataFrame(np.array(Matsf['SF_PP']['sim_time']))[0][0]
    # def cal_curve_event( sim_time ):
        curve_event = np.zeros(np.size(sim_time))
        time = np.size(curve_event) # 2399 정도 
        curve_flag =np.zeros(np.size(sim_time))
        curve_event_array = np.zeros([np.size(sim_time)])
        
        CURVATURE_REFERENCE_VALUE = 1/700
        curve_event = pd.DataFrame(curve_event)
        curve_flag =pd.DataFrame(curve_flag) 
        curve_flag_size = 0
        # curve_event.loc[1000] = 1 ## test index
        # curve_event.loc[1001] = 0
        # curve_event.loc[2000] = 1
        # curve_event.loc[2001] = 0
        # curve_event.loc[2100] = 1
        # curve_event.loc[2101] = 0

        for i in range(time):
            if (np.abs(Front_Vision_Lane_sim[CURVATURE-1,LEFT_LANE-1,i]) >CURVATURE_REFERENCE_VALUE):
                curve_event.iloc[i] = 1 # curve event occur
                # print("curve event occur!\n")#test
        
        for k in range(time):
            if (k == 0):
                curve_flag.loc[0] = curve_event.loc[0]
            
            elif (k>0):     
                if (float(curve_event.loc[k]) ==1 and float(curve_event.loc[k-1]) ==0): #Curve->Straight
                    # print('Curve->Straight\n')
                    curve_flag.loc[k] = 1
                elif (float(curve_event.loc[k]) == 0 and float(curve_event.loc[k-1]) == 1):#Straight -> Curve
                    # print('Straight -> Curve\n')
                    curve_flag.loc[k] = 2
        for i in range(time):
            if float(curve_flag.loc[i] ) >  0 :                      
                curve_flag_size +=1 
        
        return curve_flag , curve_flag_size

    def get_scenery_event(self,curve_flag , scenery_event):
        for i in range(np.size(curve_flag)):
            tmp_roadGeometry =''
            tmp_frameIndex = 0  
            if float(curve_flag.loc[i]) == 1:
                tmp_roadGeometry = 'CU'
                tmp_frameIndex = i
                scenery_event = pd.concat([scenery_event, pd.DataFrame([[tmp_frameIndex,tmp_roadGeometry]], columns=['frameIndex', 'roadGeometry'])], axis=0)
            elif float(curve_flag.loc[i]) == 2:
                tmp_roadGeometry = 'ST'
                tmp_frameIndex = i     
                scenery_event = pd.concat([scenery_event, pd.DataFrame([[tmp_frameIndex,tmp_roadGeometry]], columns=['frameIndex', 'roadGeometry'])], axis=0)  
        return scenery_event
    
    def GetRoadName(self,Regi_xlsx, num):
        dataNum = list(Regi_xlsx['dataNum'])
        roadNamelist = list(Regi_xlsx['roadName'])
        for i in range(0,np.size(dataNum),2):
            if int(dataNum[i]) <= num+1 and int(dataNum[i+1]) >= num+1 :
                roadName = roadNamelist[i]
        return roadName 

    def check_ODD(self, Regi_wrong_xlsx,num):
        ODD_STATUS = 0
        dataNum = list(Regi_wrong_xlsx['dataNum'])
        roadNamelist = list(Regi_wrong_xlsx['Description'])
        for i in range(0,np.size(dataNum)):
            if int(dataNum[i] == num+1):
                if int(roadNamelist[i]) == 7:
                    ODD_STATUS = 1
        return ODD_STATUS

    def GetParticipants(self,mat_sf,index,labelDynamic,SENSORSIZE):
        SF_PP_FUSION_TRACK_TRACKING_ID = int(np.array((mat_sf['SF_PP']['FUSION_TRACK'][0,0]['TRACKING'][0,0]['ID'][0,0]))) # 22
        SF_PP_FUSION_TRACK_VEHICLE_RECOGNITION_RECOGNITION = int(np.array((mat_sf['SF_PP']['FUSION_TRACK'][0,0]['VEHICLE_RECOGNITION'][0,0]['RECOGNITION'][0,0]))) #50
        ID = int(labelDynamic['ID'].iloc[index])
        frameIndex = int(labelDynamic['FrameIndex'].iloc[index])
        participants =[]
        tmp_arr_ID = []
        tmp_arr_recog = []
        for i in range(64):
            if frameIndex>= SENSORSIZE:
                continue
            try:
                tmp_ID = get_Fusion_Track_Maneuver(mat_sf,SF_PP_FUSION_TRACK_TRACKING_ID,i,frameIndex)
                tmp_recog = find_maneuver(get_Fusion_Track_Maneuver(mat_sf,SF_PP_FUSION_TRACK_VEHICLE_RECOGNITION_RECOGNITION,i,frameIndex))
                if ((tmp_ID > 0) and (tmp_ID) != ID):
                    if tmp_recog == 'FVL' or tmp_recog == 'FVI' or tmp_recog == 'FVR' or tmp_recog == 'AVL' or tmp_recog == 'AVR' or tmp_recog == 'RVL' or tmp_recog == 'RVI' or tmp_recog == 'RVR':
                        tmp_arr_ID.append(tmp_ID)
                        tmp_arr_recog.append(tmp_recog)
            except:
                continue
        
        size = np.size(tmp_arr_recog)
        for index in range(size):
            tmp = {
                "recognition":str(tmp_arr_recog[index]),
                "maneuver":"LK",
                "category":2,
                "participantID":int(tmp_arr_ID[index])
                }      
            participants.append(tmp)
            
        return participants

    def GetParticipantObjframe(self, index , participants,labelDynamic):
        participant_objframe = {
            "frameIndex":int(labelDynamic['FrameIndex'].iloc[index]),
            "ID":int(labelDynamic['ID'].iloc[index]),
            "participants":participants
        }
        # print("test",index)
        return participant_objframe

    def GetCSS(self, admin_dataType,admin_sampleTime,admin_version,admin_projectName,directory,driver,admin_date,admin_travelTime,admin_fileSize,admin_georeference_type,admin_georeference_coordinates,admin_CMGT,\
        admin_AESGT,admin_Stauts,admin_travelDistance,admin_annotationType,scenery,environment,dynamic,participant):

        # version 0.4 
        CSS = [{
            "dataType":admin_dataType,
            "sampleTime":admin_sampleTime,
            "version":admin_version,
            "projectName":admin_projectName, 
            "directory":directory,
            "driver":driver,
            "date":admin_date,
            "travelTime":admin_travelTime,
            "fileSize": admin_fileSize,
            "georeference":{
                "type":admin_georeference_type,
                "coordinates":admin_georeference_coordinates # 줄이 안맞음 []꼴로 나와야 하는데 줄이동이 생김 => 괜찮음 무시해도 되는 문제임
            },
            "parameter":{
                "stationaryCondition":" ",
                "trigger":" "
            },
            "CMGT":admin_CMGT,
            "AESGT":admin_AESGT,
            "Status":admin_Stauts,
            "travelDistance":admin_travelDistance, #확인 필요
            "annotationType":admin_annotationType,
            "scenery":scenery,
            "environment":environment,
            "dynamic":dynamic,
            "participant":participant,
            }]    
        
        return CSS

    def CheckCSSStatus(self, GPS_STATUS,CHASSIS_STATUS,MOBILEYE_STATUS,FRONT_RADAR_STATUS,CORNER_RADAR_STATUS,LIDAR_STATUS,ODD_STATUS,CSS_STATUS = []):
        # CSS_STATUS =[]
        if GPS_STATUS:
            GPS_STATUS = 1
            CSS_STATUS.append(GPS_STATUS)
        if CHASSIS_STATUS:
            CHASSIS_STATUS =2
            CSS_STATUS.append(CHASSIS_STATUS)
        if MOBILEYE_STATUS:
            MOBILEYE_STATUS =3
            CSS_STATUS.append(MOBILEYE_STATUS)
        if FRONT_RADAR_STATUS:
            FRONT_RADAR_STATUS = 4
            CSS_STATUS.append(FRONT_RADAR_STATUS)
        if CORNER_RADAR_STATUS:
            CORNER_RADAR_STATUS = 5
            CSS_STATUS.append(CORNER_RADAR_STATUS)                
        if LIDAR_STATUS:
            LIDAR_STATUS = 6
            CSS_STATUS.append(LIDAR_STATUS)
            
        if ODD_STATUS:
            ODD_STATUS = 7
            CSS_STATUS.append(ODD_STATUS)    
            
        if np.size(CSS_STATUS) == 0:
            CSS_STATUS.append(0)
            
        return CSS_STATUS

    def get_driver_info(self):
        driver = 10
        
        return driver


    
if __name__ == "__main__":
    dir = r"\\192.168.75.251\Shares\FOT_Avante Data_1\Rosbag2Mat\CN7_030423"
    mkjson = MakeJson(dir)
    mkjson.AutoCuration()