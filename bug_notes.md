# Bugs and Issues

Notes on bugs and issues seen during development.

## 1. Interactive Marker Reset

If you select `Enable Transparancy` under the Interactive Marker topic in Rviz BEFORE using the GUI buttons to adjust the marker's position, the marker will reset back to its origin. Once you adjust the marker through the GUI this problem seems to disappear.

## 2. Interactive Markers Branching from non Interactive Marker

If a TF is defined without an interactive marker and then a TF with is interactive marker is defined as a child of that, the interactive marker does not update properly when using the keyboard commands.

There seems to be a disconnect between the values of the imarker and the active tfs list.

### Fix

Seems that `active_tf_list_.push_back(new_tf)` was being called before `.imarker_ = true` was called. This appears to have solved the errors.

## 3. Large Decimals

On the `manipulate` tab, dragging the interactive marker causes large decimals. Need to display these only to 2 or 3 significant digits.

## 4. When `has menu` is ticked, tick `imarker` checkbox.

## 5. Multiple TFs with same name

Need to not allow user to create multiple TFs with same `to` and `from`, or create loops in TF tree. 
