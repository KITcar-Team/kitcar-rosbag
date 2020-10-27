#!/usr/bin/env python
from argparse import ArgumentParser

import progressbar  # sudo apt install python-progressbar
from rosbag import Bag
from termcolor import colored  # sudo apt install python-termcolor

parser = ArgumentParser(description="Check consistency of bagfile")
parser.add_argument("bagfile")

args = parser.parse_args()


def colorCond(cond):
    if cond:
        return "green"
    return "red"


def printColored(type, count, window):
    s = "{}\t\t[{}/{}]".format(type, count, window)
    print(colored(s, colorCond(count == window)))


with Bag(args.bagfile) as bag:
    n_messages = bag.get_message_count("/camera/image_raw")

    if n_messages == 0:
        print(colored("Bagfile does not contain any images!", "red"))
        exit()

    bar = progressbar.ProgressBar(maxval=n_messages)
    bar.start()

    i = 0
    seq = []

    print("Running bag file check:")

    for topic, msg, t in bag.read_messages(topics=["/camera/image_raw"]):
        # There should be no gaps in the sequence number
        seq.append(msg.header.seq)

        i += 1
        bar.update(i)

    bar.finish()

    print("")
    print("Details:")

    printColored("/camera/image_raw", len(seq), int(seq[-1]) - int(seq[0]) + 1)
