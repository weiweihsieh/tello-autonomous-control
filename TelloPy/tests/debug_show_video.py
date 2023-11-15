#!/usr/bin/env python2
import sys
import traceback
import tellopy
import av
import cv2
import numpy as np
import time


def main():
    drone = tellopy.Tello()
    drone.set_exposure(0)
    drone.set_video_encoder_rate(1) # 0: 16xxx-22xxx, 1: 4xxx, 2: 55xx, 3: 75xx, 4: 125xx, 5+: 16xxx # TODO: get more stats and guestimate rates, then update function docs

    try:
        drone.connect()
        drone.wait_for_connection(60.0)

        retry = 3
        container = None
        while container is None and 0 < retry:
            retry -= 1
            try:
                container = av.open(drone.get_video_stream())
            except av.AVError as ave:
                print(ave)
                print('retry...')
        # container.streams.video[0].codec_context.codec.delay = False # TODO: how can we create a new codec?

        alive = True
        while alive:
            try:
                for frame in container.decode(video=0):
                    image = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)
                    cv2.imshow('Frame', image)
                    cmd = cv2.waitKey(1) & 0xff
                    if cmd == ord('q') or cmd == ord('x'):
                        alive = False
                        break
            except av.AVError as err:
                print('AVError:',str(err))
                if str(err).find('End of file') >= 0:
                    alive = False                    

        context = container.streams.video[0].codec_context
        import pprint
        pprint.pprint({
            'codec_long_name': context.codec.long_name,
            'codec_descriptor': str(context.codec.descriptor),
            'codec_delay': str(context.codec.delay),
            'codec_dr1': str(context.codec.dr1),
            'codec_is_decoder': context.codec.is_decoder,
            'codec_is_encoder': context.codec.is_encoder,
            'codec_neg_linesizes': context.codec.neg_linesizes,
            'is_decoder': context.is_decoder,
            'is_encoder': context.is_encoder,
            'max_bit_rate': context.max_bit_rate,
            'name': context.name,
            'profile': str(context.profile),
            'options': str(context.options),
        })


    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)
    finally:
        drone.quit()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()