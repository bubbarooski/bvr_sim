#pragma once

#include <algorithm>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// A directory of type files, one subdirectory per type: <root>/<id>/<file>.
// A type is parsed the first time something asks for it and then shared --
// every entity of the type holds the same immutable T. Thread-safe, because
// bvr_batch loads scenarios on several worker threads at once.
//
// What a type file contains is the parser's business, so the JSON stays in
// the catalog's own .cpp and including this header does not pull json.hpp
// into every translation unit that uses a type.
template <typename T>
class TypeCatalog {
public:
    // Parses the type `id` from `file` (an existing file in `directory`).
    // Throws std::runtime_error naming the file if it is malformed.
    using Parser = T (*)(const std::filesystem::path& file, const std::string& id,
                         const std::filesystem::path& directory);

    // `name` prefixes every error ("AircraftCatalog"), `noun` names what is
    // missing ("aircraft type"). Relative roots resolve against the
    // process's working directory, like data/jsbsim_data.
    TypeCatalog(std::filesystem::path root, std::string fileName, std::string name, std::string noun, Parser parse)
        : root_(std::move(root)),
          fileName_(std::move(fileName)),
          name_(std::move(name)),
          noun_(std::move(noun)),
          parse_(parse) {}

    // The type named `id`, which must be one directory name under the root.
    // Throws std::runtime_error, naming the file, if there is no such type
    // or its file is malformed.
    std::shared_ptr<const T> Get(const std::string& id) const {
        // An id with a separator, a drive or ".." would climb out of the
        // root (or replace it) and load an unrelated file as a type.
        const std::filesystem::path idPath(id);
        if (id.empty() || id == "." || id == ".." || idPath.has_root_path() || idPath.has_parent_path()) {
            throw std::runtime_error(name_ + ": invalid " + noun_ + " id '" + id +
                                     "': it must be one directory name under " + root_.string());
        }
        std::lock_guard<std::mutex> lock(mutex_);
        auto found = loaded_.find(id);
        if (found != loaded_.end()) return found->second;

        std::filesystem::path directory = root_ / id;
        std::filesystem::path file = directory / fileName_;
        if (!std::filesystem::is_regular_file(file)) {
            throw std::runtime_error(name_ + ": no " + noun_ + " '" + id + "' -- expected " + file.string());
        }
        auto type = std::make_shared<const T>(parse_(file, id, directory));
        loaded_.emplace(id, type);
        return type;
    }

    // Every type id under the root, sorted.
    std::vector<std::string> Available() const {
        std::vector<std::string> ids;
        // Error-code forms throughout: an unreadable entry is skipped rather
        // than throwing out of a listing.
        std::error_code error;
        for (std::filesystem::directory_iterator it(root_, error), end; !error && it != end; it.increment(error)) {
            std::error_code entryError;
            if (it->is_directory(entryError) && std::filesystem::exists(it->path() / fileName_, entryError)) {
                ids.push_back(it->path().filename().string());
            }
        }
        std::sort(ids.begin(), ids.end());
        return ids;
    }

    const std::filesystem::path& Root() const { return root_; }

private:
    std::filesystem::path root_;
    std::string fileName_;
    std::string name_;
    std::string noun_;
    Parser parse_;
    mutable std::mutex mutex_;
    mutable std::map<std::string, std::shared_ptr<const T>> loaded_;
};
