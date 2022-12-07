from genericpath import isdir
import os
import json
import glob
import requests
import time
from pyulog.core import ULog
from collections import OrderedDict

BASE_API='https://review.px4.io/download'
DEFAULT_DOWNLOAD='dataDownloaded'
OVER_WRITE = False

def getULog(id, json_file, download_folder = DEFAULT_DOWNLOAD, overwrite=OVER_WRITE):

    with open (json_file, "r") as input_file:
        ulog_db = json.load(input_file);
        
    if not os.path.isdir(download_folder):
        os.makedirs(download_folder)
    
    logfile_pattern = os.path.join(os.path.abspath(download_folder), "*.ulg")
    log_files = glob.glob(os.path.join(os.getcwd(), logfile_pattern))
    log_ids = frozenset(os.path.splitext(os.path.basename(f))[0] for f in log_files)
    
    ulog_db = [entry for entry in ulog_db
                    if entry["id"] == id]
    num_tries = 0
    while num_tries < 99:
        try:
            if overwrite or id not in log_ids:
                file_path = os.path.join(download_folder, id + ".ulg")
                request = requests.get(url=BASE_API +
                                                "?log=" + id, stream=True)
                print(f"Downloading: {id}.ulg")
                with open(file_path, 'wb') as log_file:
                            for chunk in request.iter_content(chunk_size=1024):
                                if chunk:  # filter out keep-alive new chunks
                                    log_file.write(chunk)
            else:
                print(f"Overwrite is False -> Skipped {id}")
            break
        
        except Exception as ex:
            print(ex)
            print('Waiting for 30 seconds to retry')
            num_tries += 1
            time.sleep(30)
            
    if num_tries >= 30:
        print('Retried', str(num_tries + 1), 'times without success, exiting.')
        return False
        #  os.sys.exit(1)
    return True;
             
def removeULog(id, folder_loc=DEFAULT_DOWNLOAD):
    
    if os.path.isdir(folder_loc):
        abs_path = os.path.join(os.path.abspath(folder_loc), id + ".ulg")
        if not os.path.isfile(abs_path):
            print(f"Not a file - skipped {id}")
            return
        print(f"Deleted: {id}.ulg")
        os.remove(abs_path)
        return
    
    raise Exception("Directory Not Found")

# def extractLogData(file_path):
#     log_data = ULog(file_path)
#     data_dict = OrderedDict()
#     log_data_list = log_data.data_list

#     for d in log_data_list:
#         data_items_list = [f.field_name for f in d.field_data]
#         data_items_list.remove('timestamp')
#         data_items_list.insert(0, 'timestamp')
#         data_items = [(item, str(d.data[item].dtype), str(len(d.data[item]))) for item in data_items_list]

#         i = 0
#         name = d.name

#         while True:
#             if i > 0:
#                 name = d.name + '_' + str(i)
#             if name in data_dict:
#                 i += 1
#             else:
#                 break
#         data_dict.setdefault(name, data_items[1:])
        
#     return data_dict, log_data_list

# def getLogIds():
#     pass

# def getMultipleULogs(idList):
#     pass


# def main():
#     print("RUN")
#     getULog("df3bf3bb-5110-4507-809f-04c7f69a014c", "../QuadLogs.json", overwrite=True)
#     removeULog("df3bf3bb-5110-4507-809f-04c7f69a014c")
    
# if __name__ == "__main__":
#     main()
