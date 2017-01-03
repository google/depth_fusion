// Copyright 2016 Google Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http ://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef CONTROL_WIDGET_H
#define CONTROL_WIDGET_H

#include <QWidget>

class ControlWidget : public QWidget {

  Q_OBJECT

 public:

   ControlWidget(QWidget* parent = 0);

 signals:

   void pauseClicked();
   void stepClicked();
   void resetClicked();
   void saveMeshClicked(QString filename);
   void savePoseClicked(QString filename);

 private slots:

  void OnSaveMeshClicked();
  void OnSavePoseClicked();

};

#endif  // CONTROL_WIDGET_H
