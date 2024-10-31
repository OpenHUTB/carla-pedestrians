# 参考 https://cloud.tencent.com/developer/article/1752052

import urllib.request
import requests
import re, os
import sys

# os.environ['HTTP_PROXY'] = 'http://127.0.0.1:7890'
# os.environ['HTTPS_PROXY'] = 'https://127.0.0.1:7890'


def get_file(url):
    '''
    递归下载网站的文件
    :param url:
    :return:
    '''
    if isFile(url):
        print(url)
        try:
            download(url)
        except:
            pass
    else:
        urls = get_url(url)
        for u in urls:
            get_file(u)


def isFile(url):
    '''
    判断一个链接是否是文件
    :param url:
    :return:
    '''
    if url.endswith('/'):
        return False
    else:
        return True


def download(url):
    '''
    :param url:文件链接
    :return: 下载文件，自动创建目录
    '''
    full_name = url.split('//')[-1]
    filename = full_name.split('/')[-1]
    dirname = "/".join(full_name.split('/')[:-1])
    if os.path.exists(dirname):
        pass
    else:
        os.makedirs(dirname, exist_ok=True)
    urllib.request.urlretrieve(url, full_name)


def get_url(base_url):
    '''
    :param base_url:给定一个网址
    :return: 获取给定网址中的所有链接
    '''
    text = ''
    try:
        text = requests.get(base_url).text
    except Exception as e:
        print("error - > ",base_url,e)
        pass
    reg = '<a href="(.*)">.*</a>'
    urls = [base_url + url for url in re.findall(reg, text) if url != '../']
    return urls


if __name__ == '__main__':
    get_file('https://data.nvision2.eecs.yorku.ca/PIE_dataset/PIE_clips/')

