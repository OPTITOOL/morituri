/*
 * DummyPlugin.h
 *
 *  Created on: 11.06.2015
 *      Author: philip
 */

#ifndef DUMMYPLUGIN_HPP_
#define DUMMYPLUGIN_HPP_

#include "../base_plugin.hpp"

class dummy_plugin : public base_plugin {
public:
  dummy_plugin();
  virtual ~dummy_plugin();

  bool check_input(std::filesystem::path input_path,
                   std::filesystem::path output_path = std::filesystem::path());
  void execute();
};

#endif /* DUMMYPLUGIN_HPP_ */
