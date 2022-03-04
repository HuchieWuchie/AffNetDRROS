#!/usr/bin/env python3.6
import speech_recognition as sr
import rospy
from std_msgs.msg import Int8MultiArray, Int8


def main():

    tool_ids = {
        'bowl': 1,
        'tvm': 2,
        'pan': 3,
        'hammer': 4,
        'knife': 5,
        'cup': 6,
        'drill': 7,
        'racket': 8,
        'spatula': 9,
        'bottle': 10
    }
    affordance_id = {
        'contain': 1,
        'cut': 2,
        'display': 3,
        'engine': 4,
        'grasp': 5,
        'hit': 6,
        'pound': 7,
        'support': 8,
        'wide': 9,
        'handle': 10,
        'tool': 11,
    }

    list_microphones()

    if not rospy.is_shutdown():
        rospy.init_node('speech_recognition', anonymous=True)
        pub = rospy.Publisher('tool_id', Int8, queue_size=10)
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():

            #  Find tool from speech
            print('What object should the robot pick up?')
            found_tool = keyword_from_speech(list(tool_ids))
            tool = tool_ids[found_tool]

            #  Find affordance from speech
            #print('What affordance should the object be picked up with?')
            #found_affordance = keyword_from_speech(list(affordance_id))
            #affordance = affordance_id[found_affordance]

            #  Publish tool_id
            print(f'The robot will pick up the {found_tool}({tool})')
            while pub.get_num_connections() == 0:
                rate.sleep()
            msg = Int8()
            msg.data = tool
            pub.publish(msg)


#  Run speech_to_text until one keyword is found in speech, return keyword
def keyword_from_speech(keywords):
    found_keyword = []
    while not found_keyword:
        text = speech_to_text()
        found_keyword = keywords_from_text(text.lower(), keywords=keywords)
        if len(found_keyword) != 1:
            print('Did not find single keyword. Try again...\n')
            found_keyword = []

    return found_keyword[0]


#  Wait for speech and return as string
def speech_to_text():
    text = None
    while text is None:
        r = sr.Recognizer()
        r.energy_threshold = 40000  # Threshold for noise level to trigger recording

        with sr.Microphone() as source:
            audio = r.listen(source=source, phrase_time_limit=5)
            print('Got audio...')

        try:
            text = r.recognize_google(audio)
            print(f'Text gotten from audio: {text} \n')
        except sr.UnknownValueError:
            print('Could not get any words from audio. Try again...')
        except sr.RequestError:
            print('Could not connect to web service')

    return text


#  Look for list of keywords in string
def keywords_from_text(text, keywords):
    found_keywords = []
    for word in keywords:
        if word in text:
            found_keywords.append(word)
    return found_keywords


# List available microphones
def list_microphones():
    for index, name in enumerate(sr.Microphone.list_microphone_names()):
        print("Microphone with name \"{1}\" found for `Microphone(device_index={0})`".format(index, name))


if __name__ == '__main__':
    main()

