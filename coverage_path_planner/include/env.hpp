#ifndef ENV_H
#define ENV_H

#include <iostream>
#include <string.h>
#include <sstream>
#include <cmath>

class Env {
    public:
        static inline std::string getstring(std::string key) {
            char* str = std::getenv(key.c_str());

            if (str != NULL) {
                return str;
            }

            std::stringstream ss;
            ss << key << " was not found";

            throw new std::runtime_error(ss.str());
        }

        static inline int getint(std::string key) {
            auto str = getstring(key);

            auto i = atoi(str.c_str());

            if (std::isnan(i)) {
                std::stringstream ss;
                ss << "int isnan: " << key;
                throw new std::runtime_error(ss.str());
            }

            return i;
        }

        static inline float getfloat(std::string key) {
            auto str = getstring(key);

            auto i = atof(str.c_str());

            if (std::isnan(i)) {
                std::stringstream ss;
                ss << "int isnan: " << key;
                throw new std::runtime_error(ss.str());
            }

            return i;
        }

        static inline bool getbool(std::string key) {
            try {
                getstring(key);
                return true;
            } catch (std::runtime_error* e) {
                return false;
            }
        }
};

#endif