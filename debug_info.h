/**
   Licensed to the Apache Software Foundation (ASF) under one
   or more contributor license agreements.  See the NOTICE file
   distributed with this work for additional information
   regarding copyright ownership.  The ASF licenses this file
   to you under the Apache License, Version 2.0 (the
   "License"); you may not use this file except in compliance
   with the License.  You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
   Unless required by applicable law or agreed to in writing,
   software distributed under the License is distributed on an
   "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
   KIND, either express or implied.  See the License for the
   specific language governing permissions and limitations
   under the License.
 *******************************************************************************
 * @license Este projeto está sendo liberado pela licença APACHE 2.0.
 * @file debug_info.h
 * @brief Conjunto de protótipos e definiçoes de funções auxiliares a debug
 * @author Ânderson Ignacio da Silva
 * @date 16 Set 2017
 * @see http://www.aignacio.com
 */

#ifndef _DEBUG_INFO_H_
#define _DEBUG_INFO_H_

#define DEBUG_OS
#ifdef DEBUG_OS
#define debug_os(fmt, args ...) printf("\n[Leaf] "fmt, ## args)
#else
#define debug_os(fmt, ...)
#endif

#define DEBUG_NETWORK
#ifdef DEBUG_NETWORK
#define debug_net(fmt, args ...) printf("\n[Network] "fmt, ## args)
#else
#define debug_net(fmt, ...)
#endif

#endif
