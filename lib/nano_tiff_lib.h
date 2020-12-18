/* Header for the ultra minimalist tiff library

    Copyright 2020 Brice Dubost

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/


#ifndef nanotiff_h
#define nanotiff_h

int write_tiff(uint16_t *Tiff_image,int nx, int ny, char *filename);
int write_tiff_descr(uint16_t *Tiff_image, int nx, int ny, char *filename, char *description);


#endif
