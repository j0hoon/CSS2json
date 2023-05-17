import json
import pandas as pd
import numpy as np
import scipy.io 
import glob
import os
from tqdm import tqdm
# SELECT MODE
from datetime import datetime 
import copy
from css_frame import *

def make_CSS(directory):
    '''
    css 를 만들기 위한 함수
    
    input : directory 
    여기서 directory 는 raw data 가 있는 폴더를 의미함.
    파일 자체의 경로가 아님.
    디렉토리 구조는 아래와 같음
    - raw
        - data_name
            - data_number
                - image
                - lidar
    자세한 구조는 
    
    output : CSS.json file
    
    example : make_CSS(r"D:\Data\OP_SAMPLE\raw\multi_sensor_detection\1_1_7_20210906_010")
    '''

    
    tmp_CSS = copy.deepcopy(CSS)
    
    put_admin(tmp_CSS, directory)
    
    put_scenery(tmp_CSS, directory)
    
    
    to_json(tmp_CSS, directory)
    
    print("make_CSS")    

def put_admin(_CSS, _directory ):
    # directory
    _CSS["directory"]['raw'] = _directory

    image_directory = _directory + r"\image"
    first_image_path = glob.glob(image_directory + r"\*.jpg")[0]
    
    # date
    _CSS["date"] = str(get_file_creation_date(first_image_path)).split(".")[0]
    
    # dataType
    _CSS["dataType"] = directory.split("\\")[-1]

    # sameple time
    _CSS["sampleTime"] = "0.1"
    
    # travel Time 
    # sample time 이 있는 경우에만 계산 가능함.
    if _CSS["sampleTime"] == None:
        print("sampleTime is 0")
        travel_time = 0
    else:        
        file_num = len(glob.glob(image_directory + r"\*.jpg"))
        travel_time = file_num * float(_CSS["sampleTime"])
    _CSS["travelTime"] = str(travel_time)


def put_scenery(_CSS, _directory):
    """
    scenery 에 대한 정보를 입력하는 함수
    input : CSS, directory
    output : CSS 의 scenery 부분에 정보가 입력됨.
    
    vision only 데이터의 경우 numMaxLane 정보만 채워질 수 있음.
    
    """
    
    # lane_num_csv_path = _directory + r"\registration\registration.csv"
    lane_num_csv_path = r"D:\OneDrive - Ajou University\ACL\2_Util_Code\lane_txt_to_csv\xxxresult.csv"

    df = pd.read_csv(lane_num_csv_path)
    
    _CSS["scenery"]["numOfLane"] = [str(df["lanenumber"].min()), str(df["lanenumber"].max())]
    
    
def get_file_creation_date(file_path):
    '''
    파일의 경로를 입력해주면 그 파일이 생성된 날짜를 반환해주는 함수
    
    input : file_path
    output : creation_date ex(2021-09-06 10:00:00)
    
    example : get_file_creation_date(r"C:\Users\Administrator\OneDrive - Ajou University\바탕 화면\정영훈 전달\CN7_030323_069_Lidar.mat")
    '''
    if os.path.exists(file_path):
        timestamp = os.path.getctime(file_path)
        creation_date = datetime.fromtimestamp(timestamp)
        return creation_date
    else:
        return None    

def to_json(_CSS, _directory = os.getcwd()):
    '''
    save json file 
    input : CSS, directory
    directory : default = os.getcwd()
    
    example : to_json(CSS, r"C:\Users\Administrator\OneDrive - Ajou University\바탕 화면\정영훈 전달\CN7_030323_069_Lidar.mat")
    
    '''
    
    with open(_directory + r"\CSS.json", 'w') as outfile:
        json.dump(_CSS, outfile,indent='\t')


if __name__ == "__main__":
    os.system('cls' if os.name == 'nt' else 'clear')
    
    test = get_file_creation_date(r"C:\Users\Administrator\OneDrive - Ajou University\바탕 화면\정영훈 전달\CN7_030323_069_Lidar.mat")
    print(str(test).split(".")[0])
    
    dir_list = [r"D:\Data\OP_SAMPLE\raw\multi_sensor_detection\1_1_7_20210906_010"]
    for directory in dir_list:
        make_CSS(directory)
    print("DONE")