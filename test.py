from time import sleep


def run_sleep():
    print(f"Starting timer of 5 seconds")
    for _ in range(5):
        print(".", end="", flush=True)
        sleep(1)
    print("Done!")


if __name__ == "__main__":
    run_sleep()
