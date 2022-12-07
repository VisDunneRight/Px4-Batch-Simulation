import constants
import glob
import requests
import os
import time


class UAV_Manager:

    # def __init__(self) -> None:
    #     pass

    def download_ulog(ulog_id: str, download_folder: str = constants.DEFAULT_DOWNLOAD, overwrite: bool = False) -> bool:

        if not os.path.isdir(download_folder):
            os.makedirs(download_folder)

        logfile_pattern = os.path.join(
            os.path.abspath(download_folder), "*.ulg")
        log_files = glob.glob(os.path.join(os.getcwd(), logfile_pattern))
        log_ids = frozenset(os.path.splitext(
            os.path.basename(f))[0] for f in log_files)

        num_tries = 0
        while num_tries < 99:
            try:
                if overwrite or ulog_id not in log_ids:
                    file_path = os.path.join(download_folder, ulog_id + ".ulg")
                    request = requests.get(url=constants.BASE_URL +
                                           "?log=" + ulog_id, stream=True)
                    print(f"Downloading: {ulog_id}.ulg")
                    with open(file_path, 'wb') as log_file:
                        for chunk in request.iter_content(chunk_size=1024):
                            if chunk:  # filter out keep-alive new chunks
                                log_file.write(chunk)
                else:
                    print(f"Overwrite is False -> Skipped {ulog_id}")
                break

            except Exception as ex:
                print(ex)
                print('Waiting for 30 seconds to retry')
                num_tries += 1
                time.sleep(30)

        if num_tries >= 30:
            print('Retried', str(num_tries + 1),
                  'times without success, exiting.')
            return False
            #  os.sys.exit(1)
        return True

    def delete_ulog_file(ulog_id: str, folder_loc: str = constants.DEFAULT_DOWNLOAD) -> bool:
        if os.path.isdir(folder_loc):
            abs_path = os.path.join(
                os.path.abspath(folder_loc), ulog_id + ".ulg")
        if not os.path.isfile(abs_path):
            print(f"Not a file - skipped {ulog_id}")
            return False
        print(f"Deleted: {ulog_id}.ulg")
        os.remove(abs_path)
        return True

def main():
    u_id = "1ed4b56a-e975-4f14-99e2-d19d69df3120"
    UAV_Manager.download_ulog(u_id)
    UAV_Manager.delete_ulog_file(u_id)



if __name__ == "__main__":
    main()
