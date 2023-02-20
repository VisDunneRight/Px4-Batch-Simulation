from time import sleep


def run_sleep():
    for _ in range(5):
        print(".", end="")
        sleep(1)
    print("done!")


if __name__ == "__main__":
    run_sleep()
