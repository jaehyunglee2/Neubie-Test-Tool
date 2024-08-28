from setuptools import setup, find_packages

setup(
    name='neubie_test_tool',
    version='1.1.0',
    packages=find_packages(),
    install_requires=[
        # 여기에 패키지 의존성을 추가할 수 있습니다.
    ],
    author='Your Name',
    author_email='your.email@example.com',
    description='A description of my package',
    # url='https://github.com/yourusername/mypackage',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
)