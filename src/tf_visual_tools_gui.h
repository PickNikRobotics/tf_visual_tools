/****************************************************************************************************
 *  Software License Agreement (BSD License)
 *  
 *  Copyright 2017, Andy McEvoy
 *  
 *  Redistribution and use in source and binary forms, with or without modification, are permitted
 *  provided that the following conditions are met:
 *  
 *  1. Redistributions of source code must retain the above copyright notice, this list of conditions
 *  and the following disclaimer.
 *  
 *  2. Redistributions in binary form must reproduce the above copyright notice, this list of
 *  conditions and the following disclaimer in the documentation and/or other materials provided
 *  with the distribution.
 *  
 *  3. Neither the name of the copyright holder nor the names of its contributors may be used to
 *  endorse or promote products derived from this software without specific prior written permission.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 *  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 *  FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 *  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************************************/

/*
 * Author(s) : Andy McEvoy ( mcevoy.andy@gmail.com )
 * Desc      : Rviz display panel for dynamically manipulating TFs
 * Created   : 25 - April - 2017
 */

#ifndef TF_KEYBOARD_CAL_GUI_H
#define TF_KEYBOARD_CAL_GUI_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/package.h>
#include <rviz/panel.h>
#endif

#include <tf_visual_tools/gui_remote_receiver.h>

#include <QTabWidget>
#include <QtGui>
#include <QPushButton>
#include <QLabel>
#include <QCheckBox>
#include <QLineEdit>
#include <QComboBox>
#include <QFrame>
#include <QDoubleValidator>
#include <QDir>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QFileDialog>
#include <QGridLayout>

#include <tf/transform_datatypes.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>

#include <boost/algorithm/string.hpp>

namespace tf_visual_tools
{
struct tf_data{
  std::size_t id_;
  std::string from_;
  std::string to_;
  bool imarker_;
  QString name_;
  double values_[6];

  geometry_msgs::TransformStamped getTFMsg();
};

/**
 * Tab for manipulating TFs
 */
class manipulateTFTab : public QWidget
{
  Q_OBJECT

public:
  explicit manipulateTFTab(QWidget *parent = 0);
  void updateTFList();
  static void updateTFValues(int list_index, geometry_msgs::Pose pose);
                                                    
protected Q_SLOTS:
  void incrementDOF();
  void incrementDOF(int dof, double sign);
  
  void editTFTextValue(QString text);

  void setXYZDelta(QString text);
  void setRPYDelta(QString text);

  void setXYZDelta(double xyz_delta);
  void setRPYDelta(double rpy_delta);
  
  void setQLineValues(int item_id);

protected:
  void keyPressEvent(QKeyEvent *);
  
private:

  void updateTFValues(int dof, double value);
  
  static constexpr double MAX_XYZ_DELTA = 100.0;
  static constexpr double MAX_RPY_DELTA = 360.0;
  
  double xyz_delta_;
  double rpy_delta_;

  QComboBox *active_tfs_;
  QLineEdit *xyz_delta_box_;
  QLineEdit *rpy_delta_box_;

  std::vector<QLineEdit*> dof_qline_edits_;

  TFRemoteReceiver *remote_receiver_;
};

/**
 * Tab for creating & deleting TFs
 */
class createTFTab : public QWidget
{
  Q_OBJECT
  
public:
  explicit createTFTab(QWidget *parent = 0);
  void updateFromList();
  manipulateTFTab* manipulate_tab_;                       
  void createNewIMarker(tf_data new_tf, bool has_menu);
                                                      
protected Q_SLOTS:
  void createNewTF();
  void removeTF();

  void fromTextChanged(QString text);
  void toTextChanged(QString text);
  
private:
  void processIMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  
  bool menu_handler_set_;
  
  std::string from_tf_name_;
  std::string to_tf_name_;

  std::size_t id_;
  
  QComboBox *from_;
  QLineEdit *to_;

  QCheckBox *add_imarker_;
  QCheckBox *add_imarker_menu_;
  
  QPushButton *create_tf_btn_;
  QPushButton *remove_tf_btn_;

  QComboBox *active_tfs_;

  TFRemoteReceiver *remote_receiver_;
};

/**
 * Tab for saving and loading TFs
 */
class saveLoadTFTab : public QWidget
{
  Q_OBJECT

public:
  explicit saveLoadTFTab(QWidget *parent = 0);
  createTFTab* create_tab_;
                          
protected Q_SLOTS:
  void load();
  void save();

private:
  QPushButton *load_btn_;
  QPushButton *save_btn_;
  
  std::string full_save_path_;
  std::string full_load_path_;

  TFRemoteReceiver *remote_receiver_;
};

/**
 * Main class
 */
class TFVisualTools : public rviz::Panel
{
  Q_OBJECT
public:
  explicit TFVisualTools(QWidget *parent = 0);

protected Q_SLOTS:
  void updateTabData(int);
  
private:
  QTabWidget *tab_widget_;
  
  createTFTab *new_create_tab_;
  manipulateTFTab *new_manipulate_tab_;
  saveLoadTFTab *new_save_load_tab_;
};

} // end namespace tf_visual_tools

#endif
