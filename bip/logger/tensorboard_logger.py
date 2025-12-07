# Code referenced from https://gist.github.com/gyglim/1f8dfb1b5c82627ae3efcfbbadb9f514
import tensorflow as tf
import scipy.misc

try:
    from StringIO import StringIO  # Python 2.7
except ImportError:
    from io import BytesIO  # Python 3.x


class Logger(object):

    def __init__(self, log_dir):
        """创建一个记录到log_dir的摘要写入器。"""
        #from datetime import datetime
        #now = datetime.now()
        #log_dir = log_dir + now.strftime("%Y%m%d-%H%M%S")
        #self.writer = tf.compat.v1.summary.FileWriter(log_dir)
        self.writer = tf.summary.create_file_writer(log_dir)

    def scalar_summary(self, tag, value, step):
        """记录标量变量。"""
        #summary = tf.compat.v1.Summary(value=[tf.compat.v1.Summary.Value(tag=tag, simple_value=value)])
        #self.writer.add_summary(summary, step)

        with self.writer.as_default():
            tf.summary.scalar(tag, value, step=step)
            self.writer.flush()

    def image_summary(self, tag, images, step):
        """记录图像列表。"""

        img_summaries = []
        for i, img in enumerate(images):
            # 将图像写入字符串
            try:
                s = StringIO()
            except:
                s = BytesIO()
            scipy.misc.toimage(img).save(s, format="png")

            # 创建图像对象
            img_sum = tf.compat.v1.Summary.Image(encoded_image_string=s.getvalue(),
                                       height=img.shape[0],
                                       width=img.shape[1])

            img_summaries.append(tf.compat.v1.Summary.Value(tag='%s/%d' % (tag, i), image=img_sum))

        # 创建并撰写摘要
        summary = tf.compat.v1.Summary(value=img_summaries)
        self.writer.add_summary(summary, step)
