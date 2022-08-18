//use "g++ xxx.cpp -I/usr/include/python3.5 -lpython3.5m -o xxx" compile

//not find <Python.h>
//sudo apt-get install python-dev https://www.cnblogs.com/yuxc/archive/2012/05/13/2498019.html
//或者sudo apt-get install python3-dev
//#include <python2.7/Python.h> https://www.zhihu.com/question/392301300

#include "readPython.h"

struct position getPosition()
{

    struct position p;
    Py_Initialize();              //初始化，创建一个Python虚拟环境

    //PyRun_SimpleString("print('hello world')\n");
    
    // 2、初始化python系统文件路径，保证可以访问到 .py文件
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append('/usr/local/librealsense/examples/measure/position_trans/')");
 
    // 3、调用python文件名，不用写后缀
	PyObject* pModule = PyImport_ImportModule("trans");
	
    // 4、调用函数
	PyObject* pFunc = PyObject_GetAttrString(pModule, "rigid_transform_3D");
	
    //5、给python传参数
    // 函数调用的参数传递均是以元组的形式打包的,2表示参数个数
    // 如果AdditionFc中只有一个参数时，写1就可以了
    PyObject* pArgs = PyTuple_New(6);
 
    // 0：第一个参数，传入 int 类型的值 2
    PyTuple_SetItem(pArgs, 0, Py_BuildValue("i", 21)); 
    // 1：第二个参数，传入 int 类型的值 4
    PyTuple_SetItem(pArgs, 1, Py_BuildValue("i", 34)); 
    PyTuple_SetItem(pArgs, 2, Py_BuildValue("i", 45)); 
    PyTuple_SetItem(pArgs, 3, Py_BuildValue("i", 1)); 
    PyTuple_SetItem(pArgs, 4, Py_BuildValue("i", 2)); 
    PyTuple_SetItem(pArgs, 5, Py_BuildValue("i", 3)); 
    
    // 6、使用C++的python接口调用该函数
    PyObject* pReturn = PyEval_CallObject(pFunc, pArgs);

    // 7、接收python计算好的返回值
    int r1 = 0, r2 = 0, r3 = 0;  
    PyArg_ParseTuple(pReturn, "i|i|i", &r1, &r2, &r3);  
    if (pReturn)  
    {  
        printf("%d,%d,%d\n", r1, r2, r3); //output is 7,3  
    } 

    Py_Finalize();

    
    p.x = r1;
    p.y = r2;
    p.z = r3;
    
    return p;

}

/*int main()
{
    int *d = getPosition();
    cout << d[0] << ", " << d[1] << ", " << d[2] << endl;
}*/
