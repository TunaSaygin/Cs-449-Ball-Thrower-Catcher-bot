import robotic as ry
import time


def main():
    C = ry.Config()
    C.addFile("throwing1.g")
    C.view()
    time.sleep(30)
if __name__=="__main__":
    main()