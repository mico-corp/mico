//---------------------------------------------------------------------------------------------------------------------
//  FLOW
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#ifndef FLOW_PLUGINS_PLUGINSLOADER_H_
#define FLOW_PLUGINS_PLUGINSLOADER_H_

#include <flow/Export.h>
#include <flow/plugins/BlockPlugin.h>


namespace flow {

	class FLOW_DECL PluginsLoader {
	public:
		/// <summary>
		/// List all files in a folder and identify the ones that are dynamic libraries and try to load block plugins from them.
		/// </summary>
		/// <param name="_path"></param>
		/// <returns></returns>
		PluginNodeCreator::ListCreators parseFolder(const std::string& _path);

		/// <summary>
		/// Tries to open the library from given path and load all the block creators.
		/// </summary>
		/// <param name="_path"></param>
		/// <returns></returns>
		PluginNodeCreator::ListCreators parseLibrary(const std::string& _path);

	private:
		void readDirectory(const std::string& _path, std::vector<std::string>& _files);
		void* getHandle(const std::string& _mpluginPath);
		PluginNodeCreator* retrieveFactoryBlocks(const std::string& _path, void* _handle);

	};

}


#endif