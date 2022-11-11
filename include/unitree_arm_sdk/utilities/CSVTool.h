#ifndef CSVTOOL_H
#define CSVTOOL_H

/* 
    personal tool for .csv reading and modifying.
    only suitable for small .csv file.
    for large .csv, try <https://github.com/ben-strasser/fast-cpp-csv-parser> (read only)
 */
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include "math/typeTrans.h"

enum class FileType{
    READ_WRITE,
    CLEAR_DUMP
};

class CSVLine{
public:
    // CSVLine(std::string lineTemp, std::streampos filePos);
    CSVLine(std::string lineTemp);
    CSVLine(std::string label, std::vector<double> values);
    ~CSVLine(){}

    // void updateFilePos(std::streampos filePos){_filePos = filePos;}
    void getValues(std::vector<double> &values);
    void changeValue(std::vector<double> values);
    void writeAtEnd(std::fstream &ioStream);
    std::string getLabel(){return _label;}

private:
    // std::streampos _filePos;
    std::string _label;
    std::vector<double> _values;
};

/*
FileType::READ_WRITE : must already exist such fileName
FileType::CLEAR_DUMP : if do not exist such file, will create one
*/
class CSVTool{
public:
    CSVTool(std::string fileName, FileType type = FileType::READ_WRITE, int precision = 6);
    ~CSVTool(){_ioStream.close();}

    bool getLine(std::string label, std::vector<double> &values);
    template<typename... Args>
    bool getLineDirect(std::string label, Args&... values);

    void modifyLine(std::string label, std::vector<double> &values, bool addNew);
    template<typename... Args>
    void modifyLineDirect(std::string label, bool addNew, Args&... values);

    void readFile();
    void saveFile();

    bool _hasFile;
private:
    std::string _fileName;
    std::fstream _ioStream;
    int _precision;
    std::string _lineTemp;
    std::map<std::string, size_t> _labels;
    std::vector<CSVLine*> _lines;

};

/*************************/
/*        CSVLine        */
/*************************/
// CSVLine::CSVLine(std::string lineTemp, std::streampos filePos)
//        :_filePos(filePos){
    
//     // std::cout << lineTemp << std::endl;
// }

inline CSVLine::CSVLine(std::string lineTemp){
    // delete all spaces
    lineTemp.erase(std::remove(lineTemp.begin(), lineTemp.end(), ' '), lineTemp.end());

    std::stringstream ss(lineTemp);
    std::string stringTemp;

    std::getline(ss, _label, ',');

    while(std::getline(ss, stringTemp, ',')){
        _values.push_back(stod(stringTemp));
    }

    // std::cout << "**********" << std::endl;
    // std::cout << "_label: " << _label << std::fixed << std::setprecision(3) << std::endl;
    // for(int i(0); i<_values.size(); ++i){
    //     std::cout << _values.at(i) << ",,, ";
    // }
    // std::cout << std::endl;
}

inline CSVLine::CSVLine(std::string label, std::vector<double> values)
       :_label(label), _values(values){

}

inline void CSVLine::changeValue(std::vector<double> values){
    if(values.size() != _values.size()){
        std::cout << "[WARNING] CSVLine::changeValue, the size changed" << std::endl;
    }
    _values = values;
}

inline void CSVLine::getValues(std::vector<double> &values){
    values = _values;
}

inline void CSVLine::writeAtEnd(std::fstream &ioStream){
    ioStream << _label << ", ";

    for(int i(0); i<_values.size(); ++i){
        ioStream << _values.at(i) << ", ";
    }

    ioStream << std::endl;
}


/*************************/
/*        CSVTool        */
/*************************/
inline CSVTool::CSVTool(std::string fileName, FileType type, int precision)
       : _fileName(fileName), _precision(precision){

    if(type == FileType::READ_WRITE){
        _ioStream.open(_fileName, std::fstream::ate | std::fstream::in | std::fstream::out);

        if(!_ioStream.is_open()){
            std::cout << "[ERROR] CSVTool open file: " << fileName << " failed!" << std::endl;
            // exit(-1);
            _hasFile = false;
        }else{
            readFile();
            _hasFile = true;
        }

    }
    else if(type == FileType::CLEAR_DUMP){
        _ioStream.open(_fileName, std::fstream::out);
    }

}

inline void CSVTool::readFile(){
    if(!_ioStream.is_open()){
        // _ioStream.open(_fileName, std::fstream::ate | std::fstream::in | std::fstream::out);
        std::cout << "[ERROR] CSVTool::readFile, file: " << _fileName << " has not been opened!" << std::endl;
        return;
    }

    _lines.clear();
    _labels.clear();

    _ioStream.seekg(0, std::fstream::beg);
    size_t lineNum = 0;
    while(_ioStream && _ioStream.tellg() != std::fstream::end && getline(_ioStream, _lineTemp)){
        _lines.push_back( new CSVLine(_lineTemp) );
        
        if(_labels.count(_lines.at(lineNum)->getLabel()) == 0){
            _labels.insert(std::pair<std::string, size_t>(_lines.at(lineNum)->getLabel(), lineNum));
            ++lineNum;
        }else{
            std::cout << "[ERROR] CSVTool::readFile, the label "
                      << _lines.at(lineNum)->getLabel() << " is repeated" << std::endl;
            exit(-1);
        }
    }
}

inline bool CSVTool::getLine(std::string label, std::vector<double> &values){
    if(_labels.count(label) == 0){
        std::cout << "[ERROR] No such label: " << label << std::endl;
        return false;
    }else{
        _lines.at(_labels[label])->getValues(values);
        return true;
    }
}

template<typename... Args>
inline bool CSVTool::getLineDirect(std::string label, Args&... values){
    std::vector<double> vec;
    if(getLine(label, vec)){
        typeTrans::extractVector(vec, values...);
        return true;
    }else{
        return false;
    }
}

template<typename... Args>
inline void CSVTool::modifyLineDirect(std::string label, bool addNew, Args&... values){
    std::vector<double> vec;
    typeTrans::combineToVector(vec, values...);

// std::cout << "CSVTool::modifyLineDirect------" << std::endl;
// std::cout << "label: " << label << std::endl;
// std::cout << "vec: ";
// for(int i(0); i<vec.size(); ++i){
//     std::cout << vec.at(i) << ", ";
// }std::cout << std::endl;

    modifyLine(label, vec, addNew);
}


inline void CSVTool::saveFile(){
    _ioStream.close();
    _ioStream.open(_fileName, std::fstream::out);
    _ioStream << std::fixed << std::setprecision(_precision);
    for(int i(0); i<_lines.size(); ++i){
        _lines.at(i)->writeAtEnd(_ioStream);
    }

    _ioStream.close();
    _ioStream.open(_fileName, std::fstream::ate | std::fstream::in | std::fstream::out);
}

inline void CSVTool::modifyLine(std::string label, std::vector<double> &values, bool addNew =false){
    if(_labels.count(label) == 0){
        if(addNew){
            _labels.insert(std::pair<std::string, size_t>(label, _labels.size()));
            _lines.push_back(new CSVLine(label, values));
        }else{
            std::cout << "[ERROR] CSVTool::modifyLine, label " << label << "does not exist" << std::endl;
            exit(-1);
        }
    }else{
        _lines.at(_labels[label])->changeValue(values);
    }
}


#endif  // CSVTOOL_H