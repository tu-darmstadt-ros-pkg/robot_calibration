/*
 * Copyright (C) 2013-2014 Unbounded Robotics Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Author: Michael Ferguson

#include <ros/ros.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <string>
#include <map>
#include <tinyxml.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <robot_calibration/calibration_offset_parser.h>
#include <robot_calibration/models/chain.h>  // for rotation functions

namespace robot_calibration
{

CalibrationOffsetParser::CalibrationOffsetParser()
{
  // TODO?
}

bool CalibrationOffsetParser::add(const std::string name)
{
  parameter_names_.push_back(name);
  parameter_offsets_.push_back(0.0);
  return true;
}

bool CalibrationOffsetParser::addFrame(
    const std::string name,
    bool calibrate_x, bool calibrate_y, bool calibrate_z,
    bool calibrate_roll, bool calibrate_pitch, bool calibrate_yaw)
{
  frame_names_.push_back(name);
  if (calibrate_x)
    add(std::string(name).append("_x"));
  if (calibrate_y)
    add(std::string(name).append("_y"));
  if (calibrate_z)
    add(std::string(name).append("_z"));

  // These don't really correspond to rpy unless only one is set 
  // TODO: check that we do either roll, pitch or yaw, or all 3 (never just 2)   
  if (calibrate_roll)
    add(std::string(name).append("_a"));
  if (calibrate_pitch)
    add(std::string(name).append("_b"));
  if (calibrate_yaw)
    add(std::string(name).append("_c"));

  return true;
}

bool CalibrationOffsetParser::update(const double* const free_params)
{
  for (size_t i = 0; i < parameter_offsets_.size(); ++i)
    parameter_offsets_[i] = free_params[i];
  return true;
}

double CalibrationOffsetParser::get(const std::string name) const
{
  for (size_t i = 0; i < parameter_names_.size(); ++i)
  {
    if (parameter_names_[i] == name)
      return parameter_offsets_[i];
  }
  // Not calibrating this
  return 0.0;
}

bool CalibrationOffsetParser::getFrame(const std::string name, KDL::Frame& offset) const
{
  // Don't bother with following computation if this isn't a calibrated frame.
  bool has_offset = false;
  for (size_t i = 0; i < frame_names_.size(); ++i)
  {
    if (frame_names_[i] == name)
    {
      has_offset = true;
      break;
    }
  }
 
  if (!has_offset)
    return false;

  offset.p.x(get(std::string(name).append("_x")));
  offset.p.y(get(std::string(name).append("_y")));
  offset.p.z(get(std::string(name).append("_z")));

  offset.M = rotation_from_axis_magnitude(
                 get(std::string(name).append("_a")),
                 get(std::string(name).append("_b")),
                 get(std::string(name).append("_c")));

  return true;
}

int CalibrationOffsetParser::size()
{
  return parameter_names_.size();
}

std::string CalibrationOffsetParser::getOffsetYAML()
{
  std::stringstream ss;
  for (size_t i = 0; i < parameter_names_.size(); ++i)
  {
    ss << parameter_names_[i] << ": " << parameter_offsets_[i];
    if (parameter_offsets_[i] == 0.0) {
      ss << " [WARN] Parameter never used, check for typos!";
    }
    ss << std::endl;
  }
  return ss.str();
}

void CalibrationOffsetParser::saveOffsetYAML(std::string path)
{
  // Check if file exists
  std::fstream fstream;
  fstream.open(path, std::fstream::in | std::fstream::out);
  if (!fstream) {
    fstream.open(path,  std::fstream::in | std::fstream::out | std::fstream::trunc);
  }

  // Write calibration results
  YAML::Node offsets = YAML::LoadFile(path); // TODO check if file exists
  for (size_t i = 0; i < parameter_names_.size(); ++i)
  {
    const std::string& name = parameter_names_[i];
    const double& offset = parameter_offsets_[i];
    if (offsets[name]) {
      double new_offset = offsets[name].as<double>() + offset;
      offsets[name] = new_offset;
    } else {
      offsets[name] = offset;
    }
  }

  fstream << offsets << std::endl;
}

std::vector<std::string> split(const std::string &s, char delim) {
  std::vector<std::string> elems;
  std::stringstream ss;
  ss.str(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    if (item != "")
      elems.push_back(item);
  }
  return elems;
}

std::vector<double> splitCast(const std::string &s, char delim) {
  std::vector<double> elems;
  std::stringstream ss;
  ss.str(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    if (item != "") {
      try {
        double d_item = boost::lexical_cast<double>(item);
        elems.push_back(d_item);
      } catch (boost::bad_lexical_cast) {
        ROS_ERROR_STREAM(item << " is not a valid double.");
      }
    }
  }
  return elems;
}

std::vector<double> getVector3(TiXmlElement* xml, std::string child_name, std::string attribute_name) {
  TiXmlElement *elem_xml = xml->FirstChildElement(child_name);
  if (!elem_xml) {
    ROS_ERROR("Joint has no axis element");
    return std::vector<double>();
  }
  const std::string * attrib_str = elem_xml->Attribute(attribute_name);
  if (attrib_str == NULL) {
    ROS_ERROR_STREAM(attribute_name << "attribute of " << child_name << " element is null.");
    return std::vector<double>();
  }
  return splitCast(*attrib_str, ' ');
}

std::string CalibrationOffsetParser::getXacro(const std::string& urdf) {
  TiXmlDocument xml_doc;
  xml_doc.Parse(urdf.c_str());

  TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
  if (!robot_xml)
  {
    // We should never get here since URDF parse at beginning of calibration will fail
    ROS_ERROR("Couldn't find robot element in robot_description");
    return "";
  }

  std::stringstream file;
  // insert header
  file << "<?xml version=\"1.0\"?>" << std::endl;
  file << "<robot name=\"calibration\" xmlns:xacro=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#interface\">" << std::endl;

  /*
   * Joint offsets
   */

  // Create entry for each joint
  for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
  {
    const char * name = joint_xml->Attribute("name");

    // Is there a joint calibration needed?
    double offset = get(std::string(name));
    if (offset != 0.0)
    {
      // get rotation axis and sign
      std::vector<double> axes = getVector3(joint_xml, "axis", "xyz");
      int axis_idx = 0;
      while (axes[axis_idx] == 0.0) {
        axis_idx++;
      }
      if (axis_idx == 3) {
        ROS_ERROR("No rotation axis specified.");
        return "";
      }
      double sign = axes[axis_idx];
      double new_offset = sign * offset;
      // create entry
      file << "  <xacro:property name=\"" << name << "_offset\" value=\"" << new_offset << "\"/>" << std::endl << std::endl;
    }

    /*
     * Frame offsets
     */

    KDL::Frame frame_offset;
    bool has_update = getFrame(name, frame_offset);
    if (has_update) {
      std::vector<double> xyz_offset(3, 0.0);
      std::vector<double> rpy_offset(3, 0.0);

      xyz_offset[0] = frame_offset.p.x();
      xyz_offset[1] = frame_offset.p.y();
      xyz_offset[2] = frame_offset.p.z();

      // Get roll, pitch, yaw about fixed axis
      frame_offset.M.GetRPY(rpy_offset[0], rpy_offset[1], rpy_offset[2]);

      std::vector<double> xyz_curr = getVector3(joint_xml, "origin", "xyz");
      std::vector<double> rpy_curr = getVector3(joint_xml, "origin", "rpy");

      std::vector<double> xyz_new(3, 0.0);
      std::vector<double> rpy_new(3, 0.0);
      for (unsigned int i = 0; i < 3; i++) {
        xyz_new[i] = xyz_curr[i] + xyz_offset[i];
        rpy_new[i] = rpy_curr[i] + rpy_offset[i];
      }

      // Write into xacro
      file << "  <xacro:macro name=\"" << name << "_calibration\">" << std::endl;
      file << "    <origin xyz=\"" << xyz_new[0] << " " << xyz_new[1] << " " << xyz_new[2] << "\" rpy=\"" << rpy_new[0] << " " << rpy_new[1] << " " << rpy_new[2] << "\"/>" << std::endl;
      file << "  </xacro:macro>" << std::endl << std::endl;
    }
  }


  // close file
  file << "</robot>" << std::endl;
  return file.str();
}

std::string CalibrationOffsetParser::updateURDF(const std::string &urdf)
{
  const double precision = 8;

  TiXmlDocument xml_doc;
  xml_doc.Parse(urdf.c_str());

  TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
  if (!robot_xml)
  {
    // TODO: error notification? We should never get here since URDF parse
    //       at beginning of calibration will fail
    return urdf;
  }

  /*
   * Joint offsets
   */

  // Update each joint
  for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
  {
    const char * name = joint_xml->Attribute("name");

    // Is there a joint calibration needed?
    double offset = get(std::string(name));
    if (offset != 0.0)
    {
      TiXmlElement *calibration_xml = joint_xml->FirstChildElement("calibration");
      if (calibration_xml)
      {
        // Existing calibration, update rising attribute
        const char * rising_position_str = calibration_xml->Attribute("rising");
        if (rising_position_str != NULL)
        {
          try
          {
            offset += double(boost::lexical_cast<double>(rising_position_str));
            calibration_xml->SetDoubleAttribute("rising", offset);
          }
          catch (boost::bad_lexical_cast &e)
          {
            // TODO: error
          }
        }
        else
        {
          // TODO: error
        }
      }
      else
      {
        // No calibration previously, add an element + attribute
        calibration_xml = new TiXmlElement("calibration");
        calibration_xml->SetDoubleAttribute("rising", offset);
        TiXmlNode * calibration = calibration_xml->Clone();
        joint_xml->InsertEndChild(*calibration);
      }
    }

    /*
     * Frame offsets
     */

    KDL::Frame frame_offset;
    bool has_update = getFrame(name, frame_offset);
    if (has_update)
    {
      std::vector<double> xyz(3, 0.0);
      std::vector<double> rpy(3, 0.0);

      xyz[0] = frame_offset.p.x();
      xyz[1] = frame_offset.p.y();
      xyz[2] = frame_offset.p.z();

      // Get roll, pitch, yaw about fixed axis
      frame_offset.M.GetRPY(rpy[0], rpy[1], rpy[2]);

      // String streams for output
      std::stringstream xyz_ss, rpy_ss;

      TiXmlElement *origin_xml = joint_xml->FirstChildElement("origin");
      if (origin_xml)
      {
        // Update existing origin
        const char * xyz_str = origin_xml->Attribute("xyz");
        const char * rpy_str = origin_xml->Attribute("rpy");

        // Split out xyz of origin, break into 3 strings
        std::vector<std::string> xyz_pieces;
        boost::split(xyz_pieces, xyz_str, boost::is_any_of(" "));

        if (xyz_pieces.size() == 3)
        {
          // Update xyz
          for (int i = 0; i < 3; ++i)
          {
            double x = double(boost::lexical_cast<double>(xyz_pieces[i]) + xyz[i]);
            if (i > 0)
              xyz_ss << " ";
            xyz_ss << std::fixed << std::setprecision(precision) << x;
          }
        }
        else
        {
          // Create xyz
          for (int i = 0; i < 3; ++i)
          {
            if (i > 0)
              xyz_ss << " ";
            xyz_ss << std::fixed << std::setprecision(precision) << xyz[i];
          }
        }

        // Split out rpy of origin, break into 3 strings
        std::vector<std::string> rpy_pieces;
        boost::split(rpy_pieces, rpy_str, boost::is_any_of(" "));

        if (rpy_pieces.size() == 3)
        {
          // Update rpy
          for (int i = 0; i < 3; ++i)
          {
            double x = double(boost::lexical_cast<double>(rpy_pieces[i]) + rpy[i]);
            if (i > 0)
              rpy_ss << " ";
            rpy_ss << std::fixed << std::setprecision(precision) << x;
          }
        }
        else
        {
          // Create rpy
          for (int i = 0; i < 3; ++i)
          {
            if (i > 0)
              rpy_ss << " ";
            rpy_ss << std::fixed << std::setprecision(precision) << rpy[i];
          }
        }

        // Update xml
        origin_xml->SetAttribute("xyz", xyz_ss.str());
        origin_xml->SetAttribute("rpy", rpy_ss.str());
      }
      else
      {
        // No existing origin, create an element with attributes
        origin_xml = new TiXmlElement("origin");

        // Create xyz
        for (int i = 0; i < 3; ++i)
        {
          if (i > 0)
            xyz_ss << " ";
          xyz_ss << std::fixed << std::setprecision(precision) << xyz[i];
        }
        origin_xml->SetAttribute("xyz", xyz_ss.str());

        // Create rpy
        for (int i = 0; i < 3; ++i)
        {
          if (i > 0)
            rpy_ss << " ";
          rpy_ss << std::fixed << std::setprecision(precision) << rpy[i];
        }
        origin_xml->SetAttribute("rpy", rpy_ss.str());

        TiXmlNode * origin = origin_xml->Clone();
        joint_xml->InsertEndChild(*origin);
      }
    }
  }

  // Print to a string
  TiXmlPrinter printer;
  printer.SetIndent("  ");
  xml_doc.Accept(&printer);
  std::string new_urdf = printer.CStr();

  return new_urdf;
}

}  // namespace robot_calibration
